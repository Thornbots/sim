// Copyright 2026 Thornbots. SPDX-License-Identifier: Apache-2.0
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.CAN;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public sealed class TypeCBmi088 : ISPIPeripheral, IGPIOReceiver
    {
        public TypeCBmi088() { Reset(); }
        public void Reset()
        {
            accel = new byte[128]; gyro = new byte[128];
            accel[0] = 0x1e; gyro[0] = 0x0f; gyro[0x10] = 0x80;
            accel[0x16] = 0xaa; accel[0x17] = 0x2a;
            accel[0x22] = 17;
            accelSelected = gyroSelected = false;
            FinishTransmission();
        }
        public void OnGPIO(int number, bool value)
        {
            if(number == 0) accelSelected = !value;
            else if(number == 1) gyroSelected = !value;
            FinishTransmission();
        }
        public void FinishTransmission() { first = true; dummy = false; }
        public byte Transmit(byte data)
        {
            if(!accelSelected && !gyroSelected) return 0xff;
            var registers = accelSelected ? accel : gyro;
            if(first)
            {
                first = false; read = (data & 0x80) != 0;
                address = data & 0x7f; dummy = read && accelSelected;
                return 0;
            }
            if(dummy) { dummy = false; return 0; }
            if(read) { return registers[address++ & 0x7f]; }
            registers[address++ & 0x7f] = data;
            return 0;
        }
        public void SampleByte(int offset, byte value)
        {
            if(offset < 6) gyro[2 + offset] = value;
            else if(offset < 12) accel[0x12 + offset - 6] = value;
        }
        private byte[] accel, gyro;
        private bool first, read, dummy, accelSelected, gyroSelected;
        private int address;
    }
}
namespace Antmicro.Renode.Peripherals.Sensors
{
    public sealed class TypeCPods : Antmicro.Renode.Peripherals.I2C.II2CPeripheral
    {
        public byte[] Data = new byte[16];
        public void Reset() { Array.Clear(Data, 0, Data.Length); }
        public void Write(byte[] data) {}
        public byte[] Read(int count = 1) { var result = new byte[count]; Array.Copy(Data, result, Math.Min(count, Data.Length)); return result; }
        public void FinishTransmission() {}
    }
    public sealed class TypeCEncoder : Antmicro.Renode.Peripherals.I2C.II2CPeripheral
    {
        public ushort Angle { get; set; } = 16383;
        public void Reset() { Angle = 16383; }
        public void Write(byte[] data) { if(data.Length > 0) address = data[0]; }
        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var i = 0; i < count; i++) result[i] = (byte)(address++ == 3 ? Angle >> 6 : (Angle & 63) << 2);
            return result;
        }
        public void FinishTransmission() {}
        private int address;
    }
    public sealed class TypeCHost : Antmicro.Renode.Peripherals.Bus.IBytePeripheral, Antmicro.Renode.Peripherals.Bus.IDoubleWordPeripheral, IKnownSize
    {
        public TypeCHost(IMachine machine, Antmicro.Renode.Peripherals.UART.STM32_UART jetson,
                         Antmicro.Renode.Peripherals.UART.STM32_UART referee,
                         Antmicro.Renode.Peripherals.UART.STM32_UART dbus, TypeCPods pods, TypeCBmi088 imu, TypeCEncoder encoder)
        {
            this.jetson = jetson; this.referee = referee; this.dbus = dbus; this.pods = pods; this.imu = imu; this.encoder = encoder;
            jetson.CharReceived += value => outgoing.Enqueue(value);
            // A DR16 receiver repeats its latest 18-byte frame every 14 ms.
            remoteTimer = new LimitTimer(machine.ClockSource, 1000000, this, "dbus", limit: 14000,
                                         direction: Direction.Ascending, enabled: true, eventEnabled: true);
            remoteTimer.LimitReached += () => { if(remoteValid) foreach(var b in remote) dbus.WriteChar(b); };
        }
        public long Size => 0x1000;
        public void Reset() { outgoing.Clear(); remoteValid = false; remoteTimer.Reset(); }
        public byte ReadByte(long offset) { return outgoing.Count == 0 ? (byte)0 : outgoing.Dequeue(); }
        public uint ReadDoubleWord(long offset) { return (uint)outgoing.Count; }
        public void WriteByte(long offset, byte value)
        {
            if(offset >= 0x100 && offset < 0x200) jetson.WriteChar(value);
            else if(offset >= 0x200 && offset < 0x300) referee.WriteChar(value);
            else if(offset >= 0x300 && offset < 0x312)
            {
                // Writing the last byte starts or replaces the repeated remote frame.
                remote[offset - 0x300] = value;
                if(offset == 0x311) remoteValid = true;
            }
            else if(offset >= 0x400 && offset < 0x410) pods.Data[offset - 0x400] = value;
            else if(offset >= 0x500 && offset < 0x50c) imu.SampleByte((int)offset - 0x500, value);
            else if(offset == 0x50c) encoder.Angle = (ushort)((encoder.Angle & 0xff00) | value);
            else if(offset == 0x50d) encoder.Angle = (ushort)((encoder.Angle & 0xff) | value << 8);
        }
        public void WriteDoubleWord(long offset, uint value)
        {
            var data = BitConverter.GetBytes(value);
            for(var i = 0; i < 4; i++) WriteByte(offset + i, data[i]);
        }
        private readonly Antmicro.Renode.Peripherals.UART.STM32_UART jetson, referee, dbus;
        private readonly TypeCPods pods;
        private readonly TypeCBmi088 imu;
        private readonly TypeCEncoder encoder;
        private readonly System.Collections.Generic.Queue<byte> outgoing = new System.Collections.Generic.Queue<byte>();
        private readonly LimitTimer remoteTimer;
        private readonly byte[] remote = new byte[18];
        private bool remoteValid;
    }
}
namespace Antmicro.Renode.Peripherals.CAN
{
    // DJI motors on one CAN bus: each enabled ID 0x201..0x208 reports its latest
    // host-written 8-byte feedback once per virtual millisecond, one frame per
    // 150 us slot so the board's three-frame RX FIFO is not flooded.
    // 0x00-0x3f feedback (8 bytes per ID), 0x40 enable mask, 0x80-0x8f latest
    // 0x200/0x1ff command payloads, 0x90 command frame count.
    public sealed class TypeCMotors : ICAN, Antmicro.Renode.Peripherals.Bus.IBytePeripheral, Antmicro.Renode.Peripherals.Bus.IDoubleWordPeripheral, IKnownSize
    {
        public TypeCMotors(IMachine machine)
        {
            timer = new LimitTimer(machine.ClockSource, 1000000, this, "feedback", limit: 150,
                                   direction: Direction.Ascending, enabled: true, eventEnabled: true);
            timer.LimitReached += SendSlot;
        }
        public long Size => 0x100;
        public void Reset() { Array.Clear(registers, 0, registers.Length); slot = 0; frames = 0; timer.Reset(); }
        public byte ReadByte(long offset) { return offset < 0x90 ? registers[offset] : (byte)(frames >> (int)(8 * (offset - 0x90))); }
        public void WriteByte(long offset, byte value) { if(offset <= 0x40) registers[offset] = value; }
        public uint ReadDoubleWord(long offset) { return offset == 0x90 ? frames : BitConverter.ToUInt32(registers, (int)offset); }
        public void WriteDoubleWord(long offset, uint value)
        {
            var data = BitConverter.GetBytes(value);
            for(var i = 0; i < 4; i++) WriteByte(offset + i, data[i]);
        }
        public void OnFrameReceived(CANMessageFrame message)
        {
            var first = message.Id == 0x200 ? 0x80 : message.Id == 0x1ff ? 0x88 : -1;
            if(first < 0 || message.Data.Length != 8) return;
            Array.Copy(message.Data, 0, registers, first, 8);
            frames++;
        }
        public event Action<CANMessageFrame> FrameSent;

        private void SendSlot()
        {
            // Slots 0..5 carry frames; slot 6 pads the period to 1 ms (6 x 150 + 100 us).
            var current = slot;
            slot = (slot + 1) % 7;
            // The interval starting now ends at the next slot.
            timer.Limit = slot == 0 ? 100UL : 150UL;
            if(current == 6) return;
            var index = -1;
            for(int id = 0, seen = 0; id < 8; id++)
            {
                if((registers[0x40] & (1 << id)) == 0) continue;
                if(seen++ == current) { index = id; break; }
            }
            if(index < 0) return;
            var data = new byte[8];
            Array.Copy(registers, 8 * index, data, 0, 8);
            FrameSent?.Invoke(new CANMessageFrame((uint)(0x201 + index), data));
        }

        private readonly LimitTimer timer;
        private readonly byte[] registers = new byte[0x90];
        private int slot;
        private uint frames;
    }
}
