using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO.Ports;
using System.Diagnostics;
using System.Threading;

namespace ROS_Firmata
{
    public delegate void DidI2CDataReveive(byte address, byte register, byte[] data);
    public delegate void DigitalPinUpdated(byte pin, byte state);
    public delegate void AnalogPinUpdated(int pin, int value);

    class Arduino
    {
        public const byte LOW = 0;
        public const byte HIGH = 1;
        public const byte INPUT = 0;
        public const byte OUTPUT = 1;
        public const byte ANALOG = 2;
        public const byte PWM = 3;
        public const byte SERVO = 4;
        public const int NONE = -1;
        public const byte I2C_MODE_WRITE = 0x00;
        public const byte I2C_MODE_READ_ONCE = 0x08;
        public const byte I2C_MODE_READ_CONTINUOUSLY = 0x10;
        public const byte I2C_MODE_STOP_READING = 0x18;

        public event DidI2CDataReveive didI2CDataReveive;
        public event DigitalPinUpdated digitalPinUpdated;
        public event AnalogPinUpdated analogPinUpdated;

        // <param name="serialPortName">指定串口名称的字符串，如COM4</param>
        // <param name="baudRate">通信的波特率。默认 57600 bps（Bit Per Second）</param>
        // <param name="autoStart">确定是否应该自动打开串口。使用Open()方法为手动打开连接</param>
        // <param name="_delay">Arduino在打开串行连接后重启可能需要的时间延迟。延迟只有在自动启动为真时才会激活</param>
        public Arduino(string serialPortName, Int32 baudRate, bool autoStart, int delay)
        {
            _serialPort = new SerialPort(serialPortName, baudRate);
            _serialPort.DataBits = 8;
            _serialPort.Parity = Parity.None;
            _serialPort.StopBits = StopBits.One;

            if (autoStart)
            {
                this._delay = delay;
                this.Open();
            }
        }

        // <summary>
        //  基于用户指定的串行端口创建Arduino对象的实例
        //  默认波特率57600bps和重启延迟8秒
        //  自动打开指定的串行连接
        // </summary>
        public Arduino(string serialPortName) : this(serialPortName, 57600, true, 8000) { }

        // <summary>
        //  根据用户指定的串口和波特率创建Arduino对象的实例
        //  设定重启延迟的默认值8秒
        //  自动打开指定的串行连接
        // </summary>
        public Arduino(string serialPortName, Int32 baudRate) : this(serialPortName, baudRate, true, 8000) { }

        // <summary>
        //  使用默认参数创建Arduino对象的实例
        //  使用检测到的最高端口（即在Arduino自行识别情况下，只有一块Arduino接入的情况下）
        //  默认波特率(57600)和重启延迟(8秒)
        //  自动打开指定的串行连接
        // </summary>
        public Arduino() : this(Arduino.list().ElementAt(list().Length - 1), 57600, true, 8000) { }

        // <summary>手动打开串口连接。默认情况下，创建对象时自动打开</summary>
        public void Open()
        {
            _serialPort.DtrEnable = true;   //DTR终端启用信号为高电位
            _serialPort.Open(); //打开串口

            Thread.Sleep(_delay);   //延时等待设备重启

            byte[] command = new byte[2];

            for (int i = 0; i < 6; i++)
            {
                command[0] = (byte)(REPORT_ANALOG | i);
                command[1] = (byte)1;
                _serialPort.Write(command, 0, 2);
            }

            for (int i = 0; i < 2; i++)
            {
                command[0] = (byte)(REPORT_DIGITAL | i);
                command[1] = (byte)1;
                _serialPort.Write(command, 0, 2);
            }
            command = null;

            if (_readThread == null)
            {
                _readThread = new Thread(processInput);
                _readThread.Start();
            }
        }
        // <summary>关闭串口</summary>
        public void Close()
        {
            _readThread.Join(500);
            _readThread = null;
            _serialPort.Close();
        }

        // <summary>列出当前系统上所有可用的串口</summary>
        // <returns>包含所有可用串口的字符串数组</returns>
        public static string[] list()
        {
            return SerialPort.GetPortNames();
        }

        // <summary>设置指定引脚的模式(INPUT or OUTPUT)</summary>
        // <param name="pin">Arduino引脚</param>
        // <param name="mode">Arduino引脚模式。INPUT Arduino、OUTPUT Arduino、ANALOG Arduino、PWM、Arduino.SERVO </param>
        public void pinMode(int pin, byte mode)
        {
            byte[] message = new byte[3];
            message[0] = (byte)(SET_PIN_MODE);
            message[1] = (byte)(pin);
            message[2] = (byte)(mode);
            _serialPort.Write(message, 0, 3);
            message = null;
        }
        // <summary>读取Pin脚数字量</summary>
        // <param name="pin">Arduino数字输入引脚</param>
        // <returns>Arduino.HIGH 或者 Arduino.LOW</returns>
        public int digitalRead(int pin)
        {
            return ((_digitalInputData[pin >> 3] >> (pin & 0x07)) & 0x01);
        }

        // <summary>读取Pin脚模拟量</summary>
        // <param name="pin">Arduino模拟输入引脚</param>
        // <returns>模拟量在0 (0V)到1023 (5V)之间的一种值</returns>
        public int analogRead(int pin)
        {
            return _analogInputData[pin];
        }
        // <summary>设定输出引脚Pin的数值</summary>
        // <param name="pin">Arduino数字输出引脚</param>
        // <param name="value">值为 Arduino.LOW 或者 Arduino.HIGH.</param>
        public void digitalWrite(int pin, byte value)
        {
            int portNumber = (pin >> 3) & 0x0F;
            byte[] message = new byte[3];

            if ((int)value == 0)
                _digitalOutputData[portNumber] &= ~(1 << (pin & 0x07));
            else
                _digitalOutputData[portNumber] |= (1 << (pin & 0x07));

            message[0] = (byte)(DIGITAL_MESSAGE | portNumber);
            message[1] = (byte)(_digitalOutputData[portNumber] & 0x7F);
            message[2] = (byte)(_digitalOutputData[portNumber] >> 7);
            _serialPort.Write(message, 0, 3);
        }

        // <summary>设定模拟输出引脚（PWM）的数值</summary>
        // <param name="pin">Arduino的模拟输出引脚</param>
        // <param name="value">模拟输出范围8位精度，为0-255</param>
        public void analogWrite(int pin, int value)
        {
            byte[] message = new byte[3];
            message[0] = (byte)(ANALOG_MESSAGE | (pin & 0x0F));
            message[1] = (byte)(value & 0x7F);
            message[2] = (byte)(value >> 7);
            _serialPort.Write(message, 0, 3);
        }
        // <summary>舵机控制</summary>
        // <param name="pin">舵机引脚</param>
        // <param name="angle">舵机角度从0°到180°</param>
        public void servoWrite(int pin, int angle)
        {
            byte[] message = new byte[3];
            message[0] = (byte)(ANALOG_MESSAGE | (pin & 0x0F));
            message[1] = (byte)(angle & 0x7F);
            message[2] = (byte)(angle >> 7);
            _serialPort.Write(message, 0, 3);
        }
        // <summary>初始化IIC总线</summary>
        // <param name="delay">延时，部分IIC设备初始化需要延时</param>
        public void wireBegin(Int16 delay)
        {
            byte[] message = new byte[5];
            message[0] = (byte)(0XF0);
            message[1] = (byte)(I2C_CONFIG);
            message[2] = (byte)(delay & 0x7F);
            message[3] = (byte)(delay >> 7);
            message[4] = (byte)(END_SYSEX);//END_SYSEX
            _serialPort.Write(message, 0, 5);
        }

        // <param name="slaveAddress">IIC从机地址</param>
        // <param name="slaveRegister">数值寄存器地址</param>
        // <param name="data">写入数据或读取数据的长度</param>
        // <param name="mode">Value either Arduino.I2C_MODE_WRITE or Arduino.I2C_MODE_READ_ONCE or Arduino.I2C_MODE_READ_ONCE or Arduino.I2C_MODE_STOP_READING</param>
        public void wireRequest(byte slaveAddress, Int16 slaveRegister, Int16[] data, byte mode)
        {
            byte[] message = new byte[MAX_DATA_BYTES];
            message[0] = (byte)(0xF0);
            message[1] = (byte)(I2C_REQUEST);
            message[2] = (byte)(slaveAddress);
            message[3] = (byte)(mode);
            int index = 4;
            if (slaveRegister != Arduino.NONE)
            {
                message[index] = (byte)(slaveRegister & 0x7F);
                index += 1;
                message[index] = (byte)(slaveRegister >> 7);
                index += 1;
            }
            for (int i = 0; i < (data.Count()); i++)
            {
                message[index] = (byte)(data[i] & 0x7F);
                index += 1;
                message[index] = (byte)(data[i] >> 7);
                index += 1;
            }
            message[index] = (byte)(END_SYSEX);
            _serialPort.Write(message, 0, index + 1);
        }
        private int available()
        {
            return _serialPort.BytesToRead;
        }
        public void processInput()
        {
            while (_serialPort.IsOpen)
            {
                if (_serialPort.BytesToRead > 0)
                {
                    lock (this)
                    {
                        int inputData = _serialPort.ReadByte();
                        int command;

                        if (_parsingSysex)
                        {
                            if (inputData == END_SYSEX)
                            {
                                _parsingSysex = false;
                                if (_sysexBytesRead > 5 && _storedInputData[0] == I2C_REPLY)
                                {
                                    byte[] i2cReceivedData = new byte[(_sysexBytesRead - 1) / 2];
                                    for (int i = 0; i < i2cReceivedData.Count(); i++)
                                    {
                                        i2cReceivedData[i] = (byte)(_storedInputData[(i * 2) + 1] | _storedInputData[(i * 2) + 2] << 7);
                                    }
                                    if (this.didI2CDataReveive != null)
                                        didI2CDataReveive(i2cReceivedData[0], i2cReceivedData[1], i2cReceivedData.Skip(2).ToArray());

                                }
                                _sysexBytesRead = 0;
                            }
                            else
                            {
                                _storedInputData[_sysexBytesRead] = inputData;
                                _sysexBytesRead++;
                            }
                        }
                        else if (_waitForData > 0 && inputData < 128)
                        {
                            _waitForData--;
                            _storedInputData[_waitForData] = inputData;

                            if (_executeMultiByteCommand != 0 && _waitForData == 0)
                            {
                                switch (_executeMultiByteCommand)
                                {
                                    case DIGITAL_MESSAGE:
                                        int currentDigitalInput = (_storedInputData[0] << 7) + _storedInputData[1];
                                        for (int i = 0; i < 8; i++)
                                        {
                                            if (((1 << i) & (currentDigitalInput & 0xff)) != ((1 << i) & (_digitalInputData[_multiByteChannel] & 0xff)))
                                            {
                                                if ((((1 << i) & (currentDigitalInput & 0xff))) != 0)
                                                {
                                                    if (this.digitalPinUpdated != null)
                                                        this.digitalPinUpdated((byte)(i + _multiByteChannel * 8), Arduino.HIGH);
                                                }
                                                else
                                                {
                                                    if (this.digitalPinUpdated != null)
                                                        this.digitalPinUpdated((byte)(i + _multiByteChannel * 8), Arduino.LOW);
                                                }
                                            }
                                        }
                                        _digitalInputData[_multiByteChannel] = (_storedInputData[0] << 7) + _storedInputData[1];

                                        break;
                                    case ANALOG_MESSAGE:
                                        _analogInputData[_multiByteChannel] = (_storedInputData[0] << 7) + _storedInputData[1];
                                        if (this.analogPinUpdated != null)
                                            analogPinUpdated(_multiByteChannel, (_storedInputData[0] << 7) + _storedInputData[1]);
                                        break;
                                    case REPORT_VERSION:
                                        this._majorVersion = _storedInputData[1];
                                        this._minorVersion = _storedInputData[0];
                                        break;
                                }
                            }
                        }
                        else
                        {
                            if (inputData < 0xF0)
                            {
                                command = inputData & 0xF0;
                                _multiByteChannel = inputData & 0x0F;
                                switch (command)
                                {
                                    case DIGITAL_MESSAGE:
                                    case ANALOG_MESSAGE:
                                    case REPORT_VERSION:
                                        _waitForData = 2;
                                        _executeMultiByteCommand = command;
                                        break;
                                }
                            }
                            else if (inputData == 0xF0)
                            {
                                _parsingSysex = true;
                                // commands in the 0xF* range don't use channel data
                            }

                        }
                    }
                }
            }
        }
        #region

        private const int MAX_DATA_BYTES = 64;
        private const int TOTAL_PORTS = 2;
        private const int SERVO_CONFIG = 0x70; // set max angle, minPulse, maxPulse, freq

        private const int DIGITAL_MESSAGE = 0x90; // send data for a digital port
        private const int ANALOG_MESSAGE = 0xE0; // send data for an analog pin (or PWM)
        private const int REPORT_ANALOG = 0xC0; // enable analog input by pin #
        private const int REPORT_DIGITAL = 0xD0; // enable digital input by port
        
        private const int SET_PIN_MODE = 0xF4; // set a pin to INPUT/OUTPUT/PWM/etc
        private const int REPORT_VERSION = 0xF9; // report firmware version
        private const int SYSTEM_RESET = 0xFF; // reset from MIDI
        private const int START_SYSEX = 0xF0; // start a MIDI SysEx message
        private const int END_SYSEX = 0xF7; // end a MIDI SysEx message
        
        private const int I2C_REQUEST = 0x76; // I2C request messages from a host to an I/O board
        private const int I2C_REPLY = 0x77; // I2C reply messages from an I/O board to a host
        private const int I2C_CONFIG = 0x78; // Configure special I2C settings such as power pins and delay times
        private SerialPort _serialPort;
        private int _delay;

        private int _waitForData = 0;
        private int _executeMultiByteCommand = 0;
        private int _multiByteChannel = 0;
        private int[] _storedInputData = new int[MAX_DATA_BYTES];
        private bool _parsingSysex;
        private int _sysexBytesRead;

        private volatile int[] _digitalOutputData = new int[MAX_DATA_BYTES];
        private volatile int[] _digitalInputData = new int[MAX_DATA_BYTES];
        private volatile int[] _analogInputData = new int[MAX_DATA_BYTES];

        private int _majorVersion = 0;
        private int _minorVersion = 0;
        private Thread _readThread = null;
        private object _locker = new object();
        #endregion
    }

}

