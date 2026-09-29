// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek RTL819x SPI flash controller driver
 *
 * Copyright (c) 2026 David Yang <mmyangfl@gmail.com>
 *
 * The SPI flash controller of the RTL819x SoCs (RTL8197F and friends) is a
 * Synopsys DWC APB SSI with vendor flash extensions.  Its register layout
 * follows the APB SSI, so this driver reuses the DesignWare core for the host
 * registration, the clock divider and the chip-select handling; only the
 * transfer engine is replaced.
 *
 * It is a separate driver rather than a quirk of the generic DW mmio glue
 * because the deviations are not register tweaks: the whole mem-ops
 * implementation has to go (see the block comment below the vendor register
 * definitions).  Everything this needs is local to the IP; the generic DW
 * core, its shared header and its mmio glue are left untouched.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#include "spi-dw.h"

/* Register offsets (vendor flash extensions of the DWC APB SSI) */
#define RTL819X_SPI_ADDR_LENGTH		0x118
#define RTL819X_SPI_AUTO_LENGTH		0x11c
#define RTL819X_SPI_FLUSH_FIFO		0x128

/*
 * The transfer engine deviates from a plain DWC APB SSI in ways the
 * core-provided mem-ops cannot cope with.  Each deviation is matched against
 * the vendor driver (drivers/spi/spi-sheipa.c of the Realtek OpenWrt tree):
 *
 *   - flash extension registers at 0x118/0x11c/0x128 (address phase length,
 *     "auto" mode dummy length, FIFO flush) have no DW counterpart.  The
 *     address phase length must be reprogrammed per operation: with a stale
 *     value the chip silently ignores WRSR and erase commands, while WREN
 *     still sets WEL and reads and page programs keep working,
 *   - CTRLR1.NDF counts the frames to receive directly (an RDSR of one frame
 *     is issued with NDF = 1, enable_cs_read() -> spi_flash_setctrlr1() pass
 *     the length through), while the core programs NDF - 1,
 *   - the received data is drained blindly, exactly NDF frames after BUSY
 *     clears: spi_flash_getdr() reads the data register without ever
 *     consulting RXFLR, because RXFLR misreports the frame count on this
 *     silicon,
 *   - a transfer is never streamed: the SSI is disabled, the whole frame
 *     set is written into the TX FIFO and the SSI is enabled again.
 *     do_spi_send() splits a longer write into chunks of FIFO_HALF_SIZE (32)
 *     frames, re-issuing WREN, the program command and the address per
 *     chunk, and waits for WIP in between.  Streaming into a running FIFO
 *     silently loses data here, and a chunk has to stay within a flash page
 *     program, so the data payload is capped below the header-inclusive
 *     32 frames.
 *
 * Hence the mem-ops are replaced wholesale.  Dual/quad transfers and the
 * hardware "auto" mode (the direct flash mapping window) are not implemented.
 */
#define RTL819X_SPI_FIFO_DEPTH		64	/* 64 8-bit frames */
/*
 * One FIFO depth of received frames per transfer, as the vendor driver does:
 * NDF counts the frames exactly and the whole chunk is drained after BUSY
 * clears.
 */
#define RTL819X_SPI_RX_FRAMES_MAX	RTL819X_SPI_FIFO_DEPTH
/*
 * Data bytes per page program chunk: the vendor chunk is FIFO_HALF_SIZE (32)
 * frames, which with the opcode and a three byte address phase leaves 28
 * bytes of payload.  Every chunk is one flash command of its own, so a chunk
 * may not cross a flash page boundary; the SPI-NOR layer only hands out
 * page-aligned program operations, and the split inherits that property.
 */
#define RTL819X_SPI_WRITE_CHUNK	28

#define RTL819X_SPI_CMD_WREN		0x06
#define RTL819X_SPI_CMD_RDSR		0x05
#define RTL819X_SPI_CMD_RSTEN		0x66
#define RTL819X_SPI_CMD_RST		0x99
#define RTL819X_SPI_SR_WIP		BIT(0)

#define RTL819X_SPI_POLL_DELAY_US	1
#define RTL819X_SPI_POLL_TIMEOUT_US	100000
#define RTL819X_SPI_XFER_TIMEOUT_US	1000000

static int rtl819x_spi_wait_idle(struct dw_spi *dws)
{
	u32 val;

	return readl_poll_timeout(dws->regs + DW_SPI_SR, val,
				  !(val & DW_SPI_SR_BUSY),
				  RTL819X_SPI_POLL_DELAY_US,
				  RTL819X_SPI_POLL_TIMEOUT_US);
}

/*
 * Stop the SSI, flush the FIFOs and clear any latched error condition.
 * The DW SSI only accepts register writes once it has fully stopped
 * shifting; callers must reconfigure the controller afterwards.
 */
static void rtl819x_spi_stop(struct dw_spi *dws)
{
	dw_spi_enable_chip(dws, 0);
	rtl819x_spi_wait_idle(dws);
	dw_writel(dws, RTL819X_SPI_FLUSH_FIFO, 1);
	dw_readl(dws, DW_SPI_ICR);
}

static void rtl819x_spi_set_speed(struct dw_spi *dws, u32 speed_hz)
{
	u32 div;

	if (!speed_hz)
		speed_hz = dws->max_freq;

	/*
	 * The vendor validates the flash at spi_clk / 8 and nothing faster
	 * is known to work on this silicon.
	 */
	speed_hz = min(speed_hz, dws->max_freq / 8);

	div = DIV_ROUND_UP(dws->max_freq, speed_hz);
	div = clamp(div, 2U, 0xfffeU);
	if (div & 1)
		div++;

	dw_spi_set_clk(dws, div);
	dws->current_freq = dws->max_freq / div;
}

/*
 * Drain exactly @len received frames.  RXFLR is never consulted: after
 * BUSY clears, the FIFO is known to hold NDF frames, and RXFLR reads on
 * this silicon are unreliable (misreport when the FIFO is partially
 * drained or full), so the frames are read blindly, up to 4 frames per
 * word read with the trailing bytes taken from the last word.
 */
static void rtl819x_spi_rx_pull(struct dw_spi *dws, u8 *buf,
				unsigned int len)
{
	u32 val;

	while (len) {
		val = dw_read_io_reg(dws, DW_SPI_DR);
		if (len >= 4) {
			/* DW SSI packs the first received frame in LSB */
			buf[0] = val;
			buf[1] = val >> 8;
			buf[2] = val >> 16;
			buf[3] = val >> 24;
			buf += 4;
			len -= 4;
		} else {
			while (len) {
				*buf++ = val;
				val >>= 8;
				len--;
			}
		}
	}
}

/*
 * Push one 8-bit frame.  On this silicon a word access to the data
 * register moves up to four frames, so all transmit frames are written
 * one byte at a time.
 */
static void rtl819x_spi_tx_frame(struct dw_spi *dws, u8 val)
{
	writeb(val, dws->regs + DW_SPI_DR);
}

/*
 * Transmit the command, address and dummy cycles, which are ordinary
 * transmit frames.  In EEPROM-read mode the controller sends the
 * transmit FIFO contents first, then clocks in NDF data frames.
 */
static void rtl819x_spi_tx_header(struct dw_spi *dws,
				  const struct spi_mem_op *op, u64 addr)
{
	unsigned int i;

	rtl819x_spi_tx_frame(dws, op->cmd.opcode);
	for (i = 0; i < op->addr.nbytes; i++)
		rtl819x_spi_tx_frame(dws,
				     addr >> (8 * (op->addr.nbytes - 1 - i)));
	for (i = 0; i < op->dummy.nbytes; i++)
		rtl819x_spi_tx_frame(dws, 0);
}

static int rtl819x_spi_check_errors(struct dw_spi *dws)
{
	if (dw_readl(dws, DW_SPI_SR) & (DW_SPI_SR_TX_ERR | DW_SPI_SR_DCOL))
		return -EIO;

	return 0;
}

/*
 * Issue WREN as a bare single-frame transmit.  The SPI-NOR layer sends its own
 * WREN before an operation, so this is only needed for the chunks a write has
 * to be split into (see rtl819x_spi_write_out()).
 */
static int rtl819x_spi_wren(struct dw_spi *dws, u32 cr0)
{
	int ret;

	dw_spi_enable_chip(dws, 0);
	dw_writel(dws, DW_SPI_CTRLR0,
		  cr0 | FIELD_PREP(DW_PSSI_CTRLR0_TMOD_MASK,
				   DW_SPI_CTRLR0_TMOD_TO));
	rtl819x_spi_tx_frame(dws, RTL819X_SPI_CMD_WREN);
	dw_spi_enable_chip(dws, 1);

	ret = rtl819x_spi_wait_idle(dws);
	if (ret)
		return ret;

	return rtl819x_spi_check_errors(dws);
}

static int rtl819x_spi_wait_wip(struct dw_spi *dws, u32 cr0)
{
	unsigned int t = 100000;	/* >> worst-case page program time */
	int ret;
	u32 val;
	u8 sr;

	cr0 |= FIELD_PREP(DW_PSSI_CTRLR0_TMOD_MASK,
			  DW_SPI_CTRLR0_TMOD_EPROMREAD);
	dw_spi_enable_chip(dws, 0);
	dw_writel(dws, DW_SPI_CTRLR0, cr0);
	dw_writel(dws, DW_SPI_CTRLR1, 1);

	/*
	 * The flash needs a moment after chip select deasserts before WIP
	 * reads back as 1; an immediate poll could mistake the not yet
	 * started program for a finished one.  udelay() rather than
	 * usleep_range(): the wait is microseconds long and this SoC only
	 * gets that granularity from usleep_range() when high-resolution
	 * timers are enabled.
	 */
	udelay(50);

	do {
		dw_spi_enable_chip(dws, 0);
		rtl819x_spi_tx_frame(dws, RTL819X_SPI_CMD_RDSR);
		dw_spi_enable_chip(dws, 1);
		ret = readl_poll_timeout(dws->regs + DW_SPI_SR, val,
					 !(val & DW_SPI_SR_BUSY),
					 RTL819X_SPI_POLL_DELAY_US,
					 RTL819X_SPI_POLL_TIMEOUT_US);
		if (ret)
			return ret;
		sr = readb(dws->regs + DW_SPI_DR);
	} while ((sr & RTL819X_SPI_SR_WIP) && --t);

	dw_spi_enable_chip(dws, 0);

	return t ? 0 : -ETIMEDOUT;
}

static int rtl819x_spi_write_chunk(struct dw_spi *dws,
				   const struct spi_mem_op *op, u64 addr,
				   const u8 *buf, unsigned int len, u32 cr0)
{
	unsigned int i;
	int ret;

	dw_spi_enable_chip(dws, 0);
	dw_writel(dws, DW_SPI_CTRLR0,
		  cr0 | FIELD_PREP(DW_PSSI_CTRLR0_TMOD_MASK,
				   DW_SPI_CTRLR0_TMOD_TO));
	rtl819x_spi_tx_header(dws, op, addr);
	for (i = 0; i < len; i++)
		rtl819x_spi_tx_frame(dws, buf[i]);
	dw_spi_enable_chip(dws, 1);

	ret = rtl819x_spi_wait_idle(dws);
	if (ret)
		return ret;
	ret = rtl819x_spi_check_errors(dws);
	if (ret)
		dev_err(&dws->ctlr->dev, "write error, SR %#x\n",
			dw_readl(dws, DW_SPI_SR));

	return ret;
}

/*
 * Program @op in FIFO-half-sized chunks.  Each chunk is a program command of
 * its own: the FIFO is filled while the SSI is disabled, the transfer is
 * completed, and the flash is polled until WIP clears before the next chunk
 * is issued.
 */
static int rtl819x_spi_write_out(struct dw_spi *dws,
				 const struct spi_mem_op *op, u32 cr0)
{
	const u8 *buf = op->data.buf.out;
	u64 addr = op->addr.val;
	unsigned int done = 0;
	int ret = 0;

	while (done < op->data.nbytes) {
		unsigned int chunk = min_t(unsigned int, op->data.nbytes - done,
					   RTL819X_SPI_WRITE_CHUNK);

		if (done) {
			ret = rtl819x_spi_wren(dws, cr0);
			if (ret)
				return ret;
		}
		ret = rtl819x_spi_write_chunk(dws, op, addr, buf + done,
					      chunk, cr0);
		if (ret)
			return ret;
		ret = rtl819x_spi_wait_wip(dws, cr0);
		if (ret)
			return ret;
		done += chunk;
		addr += chunk;
	}

	return ret;
}

static int rtl819x_spi_exec_op(struct spi_mem *mem,
			       const struct spi_mem_op *op)
{
	struct spi_device *spi = mem->spi;
	struct dw_spi *dws = spi_controller_get_devdata(spi->controller);
	unsigned int pos, len;
	u64 addr = op->addr.val;
	u32 cr0, val;
	int ret;

	cr0 = FIELD_PREP(DW_PSSI_CTRLR0_DFS_MASK, 8 - 1);
	if (spi->mode & SPI_CPOL)
		cr0 |= DW_PSSI_CTRLR0_SCPOL;
	if (spi->mode & SPI_CPHA)
		cr0 |= DW_PSSI_CTRLR0_SCPHA;

	/*
	 * Default the transfer mode for the initial configuration; the
	 * mode-preserving ctrlr0 (no TMOD bits) is what the write helpers
	 * take so they can select their own transfer mode per phase.
	 */
	if (op->data.dir == SPI_MEM_DATA_IN && op->data.nbytes)
		cr0 |= FIELD_PREP(DW_PSSI_CTRLR0_TMOD_MASK,
				  DW_SPI_CTRLR0_TMOD_EPROMREAD);
	else
		cr0 |= FIELD_PREP(DW_PSSI_CTRLR0_TMOD_MASK,
				  DW_SPI_CTRLR0_TMOD_TO);

	rtl819x_spi_stop(dws);
	dw_writel(dws, DW_SPI_CTRLR0, cr0);
	rtl819x_spi_set_speed(dws, op->max_freq);
	dw_spi_set_cs(spi, false);

	/*
	 * Program the address phase length per op: address-carrying ops
	 * use op->addr.nbytes, WRSR-style ops (data out, no address) need
	 * 1 so the status byte is framed as data, and everything else
	 * (bare opcodes, EEPROM reads) is indifferent.
	 */
	dw_writel(dws, RTL819X_SPI_ADDR_LENGTH,
		  op->addr.nbytes ? op->addr.nbytes :
		  (op->data.dir == SPI_MEM_DATA_OUT ? 1 : 0));

	if (op->data.dir == SPI_MEM_DATA_OUT) {
		ret = rtl819x_spi_write_out(dws, op,
					    cr0 & ~DW_PSSI_CTRLR0_TMOD_MASK);
	} else if (op->data.dir == SPI_MEM_DATA_IN && op->data.nbytes) {
		/*
		 * Long reads are split into FIFO-sized chunks; each chunk
		 * re-issues the command header, since the chip select is
		 * deasserted while the SSI is disabled between chunks.  The
		 * vendor driver never lets a single EEPROM-read transfer
		 * exceed the FIFO depth either.
		 */
		pos = 0;
		while (pos < op->data.nbytes) {
			len = min_t(unsigned int, op->data.nbytes - pos,
				    RTL819X_SPI_RX_FRAMES_MAX);
			dw_spi_enable_chip(dws, 0);
			dw_writel(dws, DW_SPI_CTRLR0, cr0);
			/*
			 * NDF is the number of frames to clock in, matching
			 * the vendor driver (an RDSR of one frame is issued
			 * with NDF = 1).  Programming 0 here makes the
			 * controller run away clocking the maximum count,
			 * wedging the bus.
			 */
			dw_writel(dws, DW_SPI_CTRLR1, len & DW_SPI_NDF_MASK);
			rtl819x_spi_tx_header(dws, op, addr);
			dw_spi_enable_chip(dws, 1);
			/*
			 * Wait for the whole chunk to land in the FIFO
			 * before draining it, and surface controller
			 * reported errors (they would otherwise show up as
			 * a bogus rx timeout).
			 */
			ret = readl_poll_timeout(dws->regs + DW_SPI_SR, val,
						 !(val & DW_SPI_SR_BUSY),
						 RTL819X_SPI_POLL_DELAY_US,
						 RTL819X_SPI_XFER_TIMEOUT_US);
			if (ret) {
				dev_err(&spi->dev,
					"read timeout, SR %#x CTRLR1 %#x\n",
					dw_readl(dws, DW_SPI_SR),
					dw_readl(dws, DW_SPI_CTRLR1));
				goto out;
			}
			ret = rtl819x_spi_check_errors(dws);
			if (ret) {
				dev_err(&spi->dev, "read error, SR %#x\n",
					dw_readl(dws, DW_SPI_SR));
				goto out;
			}
			rtl819x_spi_rx_pull(dws, op->data.buf.in + pos,
					    len);
			pos += len;
			addr += len;
		}
	} else {
		/*
		 * Header-only transfer: command with optional address/dummy
		 * frames (WREN, WRDI, WRSR, erases, ...).
		 */
		rtl819x_spi_tx_header(dws, op, addr);
		dw_spi_enable_chip(dws, 1);
		ret = rtl819x_spi_wait_idle(dws);
		if (!ret)
			ret = rtl819x_spi_check_errors(dws);
	}

out:
	rtl819x_spi_stop(dws);
	dw_spi_set_cs(spi, true);

	return ret;
}

static bool rtl819x_spi_supports_op(struct spi_mem *mem,
				    const struct spi_mem_op *op)
{
	struct dw_spi *dws = spi_controller_get_devdata(mem->spi->controller);

	/*
	 * Only single-lane, half-duplex transfers are supported.  Writes
	 * are chunked to fit the FIFO (see rtl819x_spi_write_out), so
	 * only the command/address/dummy header must fit into the transmit
	 * FIFO.  Buswidth fields of absent phases read 0 and must not be
	 * rejected.
	 */
	if (op->cmd.buswidth != 1 ||
	    (op->addr.nbytes && op->addr.buswidth != 1) ||
	    (op->dummy.nbytes && op->dummy.buswidth != 1) ||
	    (op->data.nbytes && op->data.buswidth != 1)) {
		return false;
	}

	/* No flash write command clocks in dummy cycles. */
	if (op->data.dir == SPI_MEM_DATA_OUT && op->dummy.nbytes)
		return false;

	if (op->cmd.nbytes + op->addr.nbytes + op->dummy.nbytes >
	    dws->fifo_len) {
		return false;
	}

	return spi_mem_default_supports_op(mem, op);
}

static const struct spi_controller_mem_ops rtl819x_spi_mem_ops = {
	.supports_op = rtl819x_spi_supports_op,
	.exec_op = rtl819x_spi_exec_op,
};

/*
 * Bring the block and the flash chip up before the DW core registers the
 * host: the flash child device probes from inside that registration, so both
 * the controller configuration and the flash reset must be in place by then.
 */
static void rtl819x_spi_setup(struct dw_spi *dws)
{
	u32 cr0;

	/*
	 * The vendor does not document a Synopsys component version for
	 * this derivative; pin it below the 400A feature level so that the
	 * core does not program the CTRLR0.SPI_FRF field, which is not
	 * validated on this silicon.
	 */
	dws->ver = 1;

	/* Vendor-validated FIFO depth; do not rely on auto-detection. */
	dws->fifo_len = RTL819X_SPI_FIFO_DEPTH;
	dws->mem_ops = rtl819x_spi_mem_ops;

	cr0 = FIELD_PREP(DW_PSSI_CTRLR0_DFS_MASK, 8 - 1) |
	      FIELD_PREP(DW_PSSI_CTRLR0_TMOD_MASK, DW_SPI_CTRLR0_TMOD_TR);

	/*
	 * Stop the controller and flush the FIFOs: the boot loader leaves
	 * stale FIFO contents behind which would otherwise be shifted out
	 * as part of the first transfer.
	 */
	rtl819x_spi_stop(dws);
	/*
	 * The flash transfers run polled: leave no interrupt source
	 * enabled, the DW reset default has all of them on.
	 */
	dw_writel(dws, DW_SPI_IMR, 0);
	/* Hardware "auto" mode dummy cycles; the driver does not use it. */
	dw_writel(dws, RTL819X_SPI_AUTO_LENGTH, 0);
	dw_writel(dws, DW_SPI_CTRLR0, cr0);
	rtl819x_spi_set_speed(dws, dws->max_freq);

	/*
	 * Reset the flash chip.  The boot loader leaves it in a suspended
	 * state in which erases and status-register writes are silently
	 * ignored while reads and page programs still work; the software
	 * reset (RSTEN + RST) returns it to standby.
	 */
	dw_spi_enable_chip(dws, 0);
	dw_writel(dws, DW_SPI_CTRLR0,
		  cr0 | FIELD_PREP(DW_PSSI_CTRLR0_TMOD_MASK,
				   DW_SPI_CTRLR0_TMOD_TO));
	rtl819x_spi_tx_frame(dws, RTL819X_SPI_CMD_RSTEN);
	dw_spi_enable_chip(dws, 1);
	rtl819x_spi_wait_idle(dws);
	dw_spi_enable_chip(dws, 0);
	rtl819x_spi_tx_frame(dws, RTL819X_SPI_CMD_RST);
	dw_spi_enable_chip(dws, 1);
	rtl819x_spi_wait_idle(dws);
	dw_spi_enable_chip(dws, 0);
	dw_writel(dws, DW_SPI_CTRLR0, cr0);
	usleep_range(100, 200);
}

static int rtl819x_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct dw_spi *dws;
	struct clk *clk;
	int irq, ret;

	dws = devm_kzalloc(dev, sizeof(*dws), GFP_KERNEL);
	if (!dws)
		return -ENOMEM;

	dws->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(dws->regs))
		return PTR_ERR(dws->regs);

	dws->paddr = res->start;
	dws->reg_io_width = 4;
	dws->bus_num = pdev->id;
	device_property_read_u32(dev, "num-cs", &dws->num_cs);

	irq = platform_get_irq_optional(pdev, 0);
	if (irq == -ENXIO) {
		/*
		 * No interrupt described: run the controller polled, which
		 * the DW core supports natively.
		 */
		irq = IRQ_NOTCONNECTED;
	} else if (irq < 0) {
		return irq;
	}
	dws->irq = irq;

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to get the SPI clock\n");

	dws->max_freq = clk_get_rate(clk);
	if (!dws->max_freq)
		return dev_err_probe(dev, -EINVAL, "SPI clock has no rate\n");

	rtl819x_spi_setup(dws);

	platform_set_drvdata(pdev, dws);

	ret = dw_spi_add_controller(dev, dws);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register the SPI host\n");

	return 0;
}

static void rtl819x_spi_remove(struct platform_device *pdev)
{
	dw_spi_remove_controller(platform_get_drvdata(pdev));
}

static const struct of_device_id rtl819x_spi_of_match[] = {
	{ .compatible = "realtek,rtl8197f-spi" },
	{ /* end of table */ }
};
MODULE_DEVICE_TABLE(of, rtl819x_spi_of_match);

static struct platform_driver rtl819x_spi_driver = {
	.driver = {
		.name = "realtek-rtl819x-spi",
		.of_match_table = rtl819x_spi_of_match,
	},
	.probe = rtl819x_spi_probe,
	.remove = rtl819x_spi_remove,
};
module_platform_driver(rtl819x_spi_driver);

MODULE_DESCRIPTION("Realtek RTL819x SPI flash controller driver");
MODULE_AUTHOR("David Yang <mmyangfl@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("SPI_DW_CORE");
