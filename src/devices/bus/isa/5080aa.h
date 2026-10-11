// license:BSD-3-Clause
// copyright-holders:Chris Simmons

#ifndef MAME_BUS_ISA_5080AA_H
#define MAME_BUS_ISA_5080AA_H

#pragma once

#include "isa.h"

#include "cpu/m68000/m68010.h"
#include "imagedev/bitbngr.h"
#include "machine/mc6844.h"
#include "machine/wd1933.h"

class isa16_5080aa_device
	: public device_t
	, public device_isa16_card_interface
{
public:
	isa16_5080aa_device(machine_config const &mconfig, char const *const tag, device_t *owner, u32 clock);

	// bus master access to host memory (offset is the byte address, data is
	// little-endian); defaults to the isa memory space when not configured
	auto bus_r() { return m_bus_r.bind(); }
	auto bus_w() { return m_bus_w.bind(); }

protected:
	// device_t implementation
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	void cpu_map(address_map &map) ATTR_COLD;
	void host_io_map(address_map &map) ATTR_COLD;
	void host_mem_map(address_map &map) ATTR_COLD;

	// host interface
	void host_port_w(offs_t offset, u16 data, u16 mem_mask);
	u16 host_ram_r(offs_t offset, u16 mem_mask);
	void host_ram_w(offs_t offset, u16 data, u16 mem_mask);
	void host_interrupt();
	void status_log();

	// adapter processor
	u16 host_bus_r(offs_t offset, u16 mem_mask);
	void host_bus_w(offs_t offset, u16 data, u16 mem_mask);
	u16 parity_r(offs_t offset, u16 mem_mask);
	void parity_w(offs_t offset, u16 data, u16 mem_mask);
	void latch_w(u8 data);
	void reset_w(int state);
	void dend_w(int state);

	// serial link
	void link_tx(std::vector<u8> const &frame);
	TIMER_CALLBACK_MEMBER(link_poll);
	TIMER_CALLBACK_MEMBER(timer_tick);

private:
	required_device<m68010_device> m_cpu;
	required_device<mc6844_device> m_dma;
	required_device<wd1935_device> m_sdlc;
	required_device<bitbanger_device> m_link;
	memory_share_creator<u16> m_ram;

	devcb_read16 m_bus_r;
	devcb_write16 m_bus_w;

	emu_timer *m_timer;
	emu_timer *m_poll;

	// host interface state
	bool m_host_irq;    // adapter interrupt to host pending
	bool m_host_ie;     // adapter interrupt to host enabled
	bool m_host_int;    // isa interrupt line state
	bool m_run;         // adapter processor released from reset
	bool m_halt;        // adapter processor halted

	// adapter state
	u8 m_latch;         // line control latch
	bool m_dend_armed;  // first dend of a chained transmit is suppressed
	u8 m_dma_channel;   // channel of the current dma transfer
	u8 m_parity[256];   // bad parity flags for 0xe000-0xe0ff
	bool m_installed;

	// link framing state
	std::vector<u8> m_link_frame;
	bool m_link_escape;
	bool m_link_connected;
};

DECLARE_DEVICE_TYPE(ISA16_5080AA, isa16_5080aa_device)

#endif // MAME_BUS_ISA_5080AA_H
