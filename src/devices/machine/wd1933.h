// license:BSD-3-Clause
// copyright-holders:Chris Simmons

#ifndef MAME_MACHINE_WD1933_H
#define MAME_MACHINE_WD1933_H

#pragma once

#include <deque>
#include <vector>

class wd1933_device : public device_t
{
public:
	using frame_delegate = device_delegate<void (std::vector<u8> const &frame)>;

	wd1933_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);

	// clock is the serial bit rate (1X clock)
	auto out_intrq() { return m_out_intrq.bind(); }
	auto out_drqo() { return m_out_drqo.bind(); }
	auto out_drqi() { return m_out_drqi.bind(); }
	auto out_rts() { return m_out_rts.bind(); }
	auto out_dtr() { return m_out_dtr.bind(); }
	auto out_misc() { return m_out_misc.bind(); }

	// called with the address, control, and information fields of each
	// complete frame transmitted to the line (excludes FCS and flags)
	template <typename... T> void set_frame_tx_callback(T &&... args) { m_frame_tx.set(std::forward<T>(args)...); }

	// register access (offset is A2-A0 as defined in the data sheet)
	u8 read(offs_t offset);
	void write(offs_t offset, u8 data);

	// end of block input (active high, equivalent to an fcs command)
	void eob_w(int state);

	// modem control inputs (active high)
	void dsr_w(int state);
	void cd_w(int state);
	void ri_w(int state);
	void misc_w(int state);

	// queue a frame received from the line (address, control, and
	// information fields, excluding FCS and flags)
	void rx_frame(u8 const *data, unsigned length);

	// character-level line interface: the transmitter output is presented
	// on out_line() as each character (or frame end/abort) is sent, and may
	// be fed back to line_w() to model an external wrap
	enum line_event : u16
	{
		LINE_BYTE  = 0x000, // data character in bits 0-7
		LINE_END   = 0x100, // closing flag
		LINE_ABORT = 0x200, // abort
	};
	auto out_line() { return m_out_line.bind(); }
	void line_w(u16 event);

protected:
	wd1933_device(machine_config const &mconfig, device_type type, char const *tag, device_t *owner, u32 clock);

	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	TIMER_CALLBACK_MEMBER(tx_tick);
	TIMER_CALLBACK_MEMBER(rx_tick);

private:
	enum cr1_mask : u8
	{
		CR1_MISC = 0x01, // miscellaneous output
		CR1_DTR  = 0x02, // data terminal ready
		CR1_TCL  = 0x0c, // transmit character length
		CR1_TC   = 0x30, // transmit command
		CR1_ACTT = 0x40, // activate transmitter
		CR1_ACTR = 0x80, // activate receiver
	};
	enum cr2_mask : u8
	{
		CR2_AF   = 0x01, // auto flag
		CR2_ST   = 0x02, // self-test
		CR2_LOOP = 0x04, // sdlc loop mode
		CR2_RCL  = 0x18, // receive character length
		CR2_EA   = 0x20, // extended address
		CR2_AC   = 0x40, // address compare
		CR2_EC   = 0x80, // extended control
	};
	enum tc : u8
	{
		TC_DATA  = 0,
		TC_ABORT = 1,
		TC_FLAG  = 2,
		TC_FCS   = 3,
	};
	enum ir_mask : u8
	{
		IR_INTRQ = 0x01,
		IR_DRQO  = 0x02,
		IR_DRQI  = 0x04,
		IR_DSC   = 0x08, // data set change
		IR_XUND  = 0x10, // transmit complete with underrun
		IR_XOK   = 0x20, // transmit complete
		IR_REOME = 0x40, // received end of message with errors
		IR_REOM  = 0x80, // received end of message
	};
	enum sr_mask : u8
	{
		SR_CRC   = 0x01,
		SR_OVR   = 0x02,
		SR_ABT   = 0x04, // aborted or invalid frame
		SR_IDLE  = 0x08, // receiver idle
		SR_MISC  = 0x10,
		SR_DSR   = 0x20,
		SR_CD    = 0x40,
		SR_RI    = 0x80,
	};

	enum tx_state : u8
	{
		TX_IDLE,
		TX_FRAME,
		TX_FCS,
	};
	enum rx_state : u8
	{
		RX_HUNT,
		RX_FRAME,
		RX_DISCARD,
	};
	enum rx_event : u16
	{
		RX_BYTE  = 0x000,
		RX_END   = 0x100,
		RX_ABORT = 0x200,
	};

	unsigned tc() const { return (m_cr1 & CR1_TC) >> 4; }
	bool self_test() const { return m_cr2 & CR2_ST; }
	attotime char_time() const { return attotime::from_hz(clock()) * 8; }

	void cr1_w(u8 data);
	void cr2_w(u8 data);
	void thr_w(u8 data);

	void set_drqo(bool state);
	void set_drqi(bool state);
	void set_ir(u8 mask);
	void update_intrq();
	void modem_change();

	void tx_shift();
	void tx_line(u16 event);
	void tx_abort();

	void rx_event(u16 event);

	static u16 crc_update(u16 crc, u8 data);

	devcb_write_line m_out_intrq;
	devcb_write_line m_out_drqo;
	devcb_write_line m_out_drqi;
	devcb_write_line m_out_rts;
	devcb_write_line m_out_dtr;
	devcb_write_line m_out_misc;
	devcb_write16 m_out_line;
	frame_delegate m_frame_tx;

	emu_timer *m_tx_timer;
	emu_timer *m_rx_timer;

	// registers
	u8 m_cr1;
	u8 m_cr2;
	u8 m_cr3;
	u8 m_ar;
	u8 m_rhr;
	u8 m_thr;
	u8 m_ir;
	u8 m_sr;

	// output state
	bool m_intrq;
	bool m_drqo;
	bool m_drqi;

	// input state
	bool m_eob;
	bool m_dsr;
	bool m_cd;
	bool m_ri;
	bool m_misc;

	// transmitter state
	u8 m_tx_state;
	bool m_thr_full;
	bool m_tx_cmd;      // non-data command awaiting completion
	unsigned m_tx_fcs;  // fcs/flag characters remaining
	u16 m_tx_crc;
	std::vector<u8> m_tx_frame;

	// receiver state
	u8 m_rx_state;
	bool m_rx_ovr;
	unsigned m_rx_count;
	u16 m_rx_crc;
	std::deque<u16> m_rx_queue;
};

class wd1935_device : public wd1933_device
{
public:
	wd1935_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);
};

DECLARE_DEVICE_TYPE(WD1933, wd1933_device)
DECLARE_DEVICE_TYPE(WD1935, wd1935_device)

#endif // MAME_MACHINE_WD1933_H
