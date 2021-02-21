using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

using ROS_Firmata;

namespace Moveit
{
    public partial class Form1 : Form
    {
        Arduino arduino = new Arduino("COM3",115200);

        public int joint1 = 3;
        public int joint2 = 5;
        public int joint3 = 6;
        public int joint4 = 9;
        public int joint5 = 10;
        public int joint6 = 11;

        public int joint1_value = 90;
        public int joint2_value = 90;
        public int joint3_value = 90;
        public int joint4_value = 90;
        public int joint5_value = 90;
        public int joint6_value = 10;
        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            this.FormClosing += Form1_FormClosing;

            arduino.pinMode(joint1, Arduino.SERVO);
            arduino.pinMode(joint2, Arduino.SERVO);
            arduino.pinMode(joint3, Arduino.SERVO);
            arduino.pinMode(joint4, Arduino.SERVO);
            arduino.pinMode(joint5, Arduino.SERVO);
            arduino.pinMode(joint6, Arduino.SERVO);

            arduino.servoWrite(joint1, joint1_value);
            arduino.servoWrite(joint2, joint2_value);
            arduino.servoWrite(joint3, joint3_value);
            arduino.servoWrite(joint4, joint4_value);
            arduino.servoWrite(joint5, joint5_value);
            arduino.servoWrite(joint6, joint6_value);

            textBox1.Text = Convert.ToString(joint1_value);
            textBox2.Text = Convert.ToString(joint2_value);
            textBox3.Text = Convert.ToString(joint3_value);
            textBox4.Text = Convert.ToString(joint4_value);
            textBox5.Text = Convert.ToString(joint5_value);
            textBox6.Text = Convert.ToString(joint6_value);

            textBox7.Text = Convert.ToString(joint1_value);
            textBox8.Text = Convert.ToString(joint2_value);
            textBox9.Text = Convert.ToString(joint3_value);
            textBox10.Text = Convert.ToString(joint4_value);
            textBox11.Text = Convert.ToString(joint5_value);
            textBox12.Text = Convert.ToString(joint6_value);
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            arduino.Close();
        }

        private void trackBar1_Scroll(object sender, EventArgs e)
        {
            TrackBar bar = (TrackBar)(sender);
            arduino.servoWrite(joint1, bar.Value);
            textBox1.Text = Convert.ToString(bar.Value);
        }

        private void trackBar2_Scroll(object sender, EventArgs e)
        {
            TrackBar bar = (TrackBar)(sender);
            arduino.servoWrite(joint2, bar.Value);
            textBox2.Text = Convert.ToString(bar.Value);
        }

        private void trackBar3_Scroll(object sender, EventArgs e)
        {
            TrackBar bar = (TrackBar)(sender);
            arduino.servoWrite(joint3, bar.Value);
            textBox3.Text = Convert.ToString(bar.Value);
        }

        private void trackBar4_Scroll(object sender, EventArgs e)
        {
            TrackBar bar = (TrackBar)(sender);
            arduino.servoWrite(joint4, bar.Value);
            textBox4.Text = Convert.ToString(bar.Value);
        }

        private void trackBar5_Scroll(object sender, EventArgs e)
        {
            TrackBar bar = (TrackBar)(sender);
            arduino.servoWrite(joint5, bar.Value);
            textBox5.Text = Convert.ToString(bar.Value);
        }

        private void trackBar6_Scroll(object sender, EventArgs e)
        {
            TrackBar bar = (TrackBar)(sender);
            arduino.servoWrite(joint6, bar.Value);
            textBox6.Text = Convert.ToString(bar.Value);
        }

        private void button1_Click(object sender, EventArgs e)
        {
            arduino.servoWrite(joint1, joint1_value);
            arduino.servoWrite(joint2, joint2_value);
            arduino.servoWrite(joint3, joint3_value);
            arduino.servoWrite(joint4, joint4_value);
            arduino.servoWrite(joint5, joint5_value);
            arduino.servoWrite(joint6, joint6_value);

            textBox1.Text = Convert.ToString(joint1_value);
            textBox2.Text = Convert.ToString(joint2_value);
            textBox3.Text = Convert.ToString(joint3_value);
            textBox4.Text = Convert.ToString(joint4_value);
            textBox5.Text = Convert.ToString(joint5_value);
            textBox6.Text = Convert.ToString(joint6_value);

            trackBar1.Value = joint1_value;
            trackBar2.Value = joint2_value;
            trackBar3.Value = joint3_value;
            trackBar4.Value = joint4_value;
            trackBar5.Value = joint5_value;
            trackBar6.Value = joint6_value;
        }

        private void button2_Click(object sender, EventArgs e)
        {
            arduino.servoWrite(joint1, Convert.ToInt32(textBox7.Text));
            arduino.servoWrite(joint2, Convert.ToInt32(textBox8.Text));
            arduino.servoWrite(joint3, Convert.ToInt32(textBox9.Text));
            arduino.servoWrite(joint4, Convert.ToInt32(textBox10.Text));
            arduino.servoWrite(joint5, Convert.ToInt32(textBox11.Text));
            arduino.servoWrite(joint6, Convert.ToInt32(textBox12.Text));

            trackBar1.Value = Convert.ToInt32(textBox7.Text);
            trackBar2.Value = Convert.ToInt32(textBox8.Text);
            trackBar3.Value = Convert.ToInt32(textBox9.Text);
            trackBar4.Value = Convert.ToInt32(textBox10.Text);
            trackBar5.Value = Convert.ToInt32(textBox11.Text);
            trackBar6.Value = Convert.ToInt32(textBox12.Text);

            textBox1.Text = textBox7.Text;
            textBox2.Text = textBox8.Text;
            textBox3.Text = textBox9.Text;
            textBox4.Text = textBox10.Text;
            textBox5.Text = textBox11.Text;
            textBox6.Text = textBox12.Text;
        }
    }
}
