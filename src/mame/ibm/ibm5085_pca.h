
#ifndef MAME_IBM_IBM5085_PCA_H
#define MAME_IBM_IBM5085_PCA_H

#pragma once

#include "emu.h"
#include "bus/rs232/rs232.h"
#include "rtpc_kbdc.h"

class ibm5085_pca_device : public buffered_rs232_device<16>
{
public:
	ibm5085_pca_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual void received_byte(u8 data) override;   // a byte from the AP

private:
	void kbd_clock_w(int state);                     // AT clock edges from the connector
	void kbd_send(u8 data);                          // host -> keyboard
	TIMER_CALLBACK_MEMBER(kbd_request_to_send);
	void kbd_received(u8 data);                      // keyboard -> host

	required_device<rtpc_kbdc_device> m_kbdc;
	emu_timer *m_rts_timer;

	// AT host shift register
	enum : u8 { IDLE, RX, TX_HOLD, TX };
	u8 m_state;
	u16 m_shift;
	int m_bit;

	bool m_loopback;
	u8 m_config;
};

DECLARE_DEVICE_TYPE(IBM5085_PCA, ibm5085_pca_device)

#endif // MAME_IBM_IBM5085_PCA_H
