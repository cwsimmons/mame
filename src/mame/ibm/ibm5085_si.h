
#ifndef MAME_IBM_IBM5085_SI_H
#define MAME_IBM_IBM5085_SI_H

#pragma once

#include "cpu/m68000/m68010.h"

class ibm5085_si_device : public device_t
{
public:
	ibm5085_si_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock = 0);

protected:
	// device_t implementation
	virtual tiny_rom_entry const *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	void cpu_map(address_map &map) ATTR_COLD;

	required_device<m68010_device> m_cpu;
	required_region_ptr<u16> m_rom;
    memory_view m_bootvect;

	emu_timer *m_boot_timer = nullptr;
	TIMER_CALLBACK_MEMBER(boot_timeout);

    void doorbell_w(u8 data);
};

DECLARE_DEVICE_TYPE(IBM5085_SI, ibm5085_si_device)

#endif // MAME_IBM_IBM5085_SI_H
