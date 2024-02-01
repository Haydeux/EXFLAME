import wx
from threading import Thread
from wx.lib.plot import PlotCanvas, PlotGraphics, PolySpline, PolyMarker
import numpy as np
import math
import socket

class ethernet_client:
    PORT = 1050                     # Port that the arduino is listening on (Must match the Port in the .ino code)
    IP_ADDR = "192.168.1.102"       # IP address of the arduino (Must match the IP being used in the .ino code)

    def __init__(self, callback_func = None):
        server_addr = (self.IP_ADDR, self.PORT)
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(server_addr)
        self.client_socket.sendall(b'\n')
        connect_msg = self.client_socket.recv(24)
        print(connect_msg.decode("utf-8"), sep="", end="")

        self.callback_func = callback_func
        self.thread = None
        self.alive = False

    def send_msg(self, msg: str):
        msg_b = msg.encode("utf-8")
        self.client_socket.sendall(msg_b)

    def recieve_msg(self):
        msg = ""
        charc = ''
        while ((not ((charc == '\n') or (charc == '\b'))) and (self.alive)):
            data = self.client_socket.recv(1)
            charc = data.decode('utf-8')
            msg += charc
        return msg

    def close(self):
        self.client_socket.send(b'dc\n')
        dc_msg = self.client_socket.recv(24)
        print(dc_msg.decode("utf-8"), sep="", end="")
        if self.alive:
            self.stop_reader()
        self.client_socket.close()

    def start_reader(self):
        self.alive = True
        self.thread = Thread(target = self.msg_reciever)
        self.thread.daemon = True
        self.thread.start()

    def stop_reader(self):
        if (self.thread is not None):
            self.alive = False
            self.thread.join()
            self.thread = None

    def msg_reciever(self):
        while self.alive:
            msg = self.recieve_msg()
            if ((msg != "") and (self.callback_func is not None) and self.alive):
                # return value to main loop in thread-safe manner
                wx.CallAfter(self.callback_func, msg)

# Window that pops up when advanced buton is pressed to alter the crank parameters
class AdvancedWindow(wx.Frame):
    # Initialise
    def __init__(self, parent):
        super(AdvancedWindow, self).__init__(parent, title='Advanced Settings', size=(340, 200))
        self.parent = parent
        self.adv_lock = False

        # Initialise panel and box sizer 
        self.panel = wx.Panel(self)
        self.big_sizer = wx.BoxSizer(wx.HORIZONTAL)

        # Layout widgets
        self.init_adv_col_1()
        self.init_adv_col_2()

        # Add sizers to big_sizer
        self.big_sizer.Add(self.adv_col_1_sizer)
        self.big_sizer.Add(self.adv_col_2_sizer)

        # Layout panel
        self.panel.SetSizerAndFit(self.big_sizer)
        self.Centre()

    # Initialise a column of labels
    def init_adv_col_1(self):
        # Sizer to space out labels
        self.adv_col_1_sizer = wx.BoxSizer(wx.VERTICAL) 

        # Crank A Amplitude Label
        line1= wx.BoxSizer(wx.HORIZONTAL) 
        l1_label1 = wx.StaticText(self.panel, -1, "Crank A Amplitude") 
        line1.Add(l1_label1, 0, wx.EXPAND|wx.CENTER|wx.ALL,5)
        self.adv_col_1_sizer.Add(line1)    

        # Crank B Amplitude Label
        line2 = wx.BoxSizer(wx.HORIZONTAL) 
        l2_label1 = wx.StaticText(self.panel, -1, "Crank B Amplitude") 
        line2.Add(l2_label1, 1, wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,8)
        self.adv_col_1_sizer.Add(line2)

        # Arm Length Label
        line3 = wx.BoxSizer(wx.HORIZONTAL) 
        l3_label1 = wx.StaticText(self.panel, -1, "Arm Length") 
        line3.Add(l3_label1, 1, wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,8)
        self.adv_col_1_sizer.Add(line3)

        # Encoder Offset Label
        line4 = wx.BoxSizer(wx.HORIZONTAL) 
        l4_label1 = wx.StaticText(self.panel, -1, "Absolute Encoder Offset") 
        line4.Add(l4_label1, 1, wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,8)
        self.adv_col_1_sizer.Add(line4)

    # Initialise a column of text boxes and some buttons
    def init_adv_col_2(self):
        # Sizer to space out text boxes and buttons
        self.adv_col_2_sizer = wx.BoxSizer(wx.VERTICAL) 

        # Edit Crank A Amplitude
        self.crank_a_amplitude_set = wx.TextCtrl(self.panel, value = str(self.parent.crank_a_amplitude), style = wx.TE_READONLY|wx.TE_CENTER) 
        self.adv_col_2_sizer.Add(self.crank_a_amplitude_set,1,wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,5) 

        # Edit Crank B Amplitude
        self.crank_b_amplitude_set = wx.TextCtrl(self.panel, value = str(self.parent.crank_b_amplitude), style = wx.TE_READONLY|wx.TE_CENTER) 
        self.adv_col_2_sizer.Add(self.crank_b_amplitude_set,1,wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,5) 

        # Edit Arm Length
        self.arm_length_set = wx.TextCtrl(self.panel, value = str(self.parent.arm_length), style = wx.TE_READONLY|wx.TE_CENTER) 
        self.adv_col_2_sizer.Add(self.arm_length_set,1,wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,5) 

        # Edit Encoder Absolute Offset
        self.encoder_absolute_difference_set = wx.TextCtrl(self.panel, value = str(self.parent.encoder_absolute_difference), style = wx.TE_READONLY|wx.TE_CENTER) 
        self.adv_col_2_sizer.Add(self.encoder_absolute_difference_set,1,wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,5) 

        # Create another box sizer for spacing buttons
        adv_btns = wx.BoxSizer(wx.HORIZONTAL)

        # Lock btn for setting editability
        self.adv_lock_btn = wx.Button(self.panel, label='Unlock')
        self.adv_lock_btn.Bind(wx.EVT_BUTTON, self.on_press_adv_lock)
        adv_btns.Add(self.adv_lock_btn, 0, wx.ALL | wx.LEFT, 5)

        # Apply changes
        self.adv_apply_btn = wx.Button(self.panel, label='Apply')
        self.adv_apply_btn.Bind(wx.EVT_BUTTON, self.on_press_adv_apply)
        adv_btns.Add(self.adv_apply_btn, 0, wx.ALL, 5)
        self.adv_col_2_sizer.Add(adv_btns)

    # Check if entered value is valid
    def check_if_in_range(self, val, min, max):
        try:
            val_float = float(val)
            return min <= val_float <= max
        except ValueError:
            return False

    # Locking functionality
    def on_press_adv_lock(self, event):
        # Change editability of target inputs
        self.adv_lock = not self.adv_lock
        self.crank_a_amplitude_set.SetEditable(self.adv_lock)
        self.crank_b_amplitude_set.SetEditable(self.adv_lock)
        self.arm_length_set.SetEditable(self.adv_lock)
        self.encoder_absolute_difference_set.SetEditable(self.adv_lock)

        # Change button name
        if (self.adv_lock == False):
            self.adv_lock_btn.SetLabel("Unlock")
        else:
            self.adv_lock_btn.SetLabel("Lock")

    def get_absolute_setpoint_difference(self, amp_a, amp_b):
        offset_a = self.get_offset_from_amplitude(amp_a)
        offset_b = self.get_offset_from_amplitude(amp_b)
        overall_offset = (offset_b - offset_a) % self.parent.ppr
        print("ofa ", offset_a)
        print("offb ", offset_b)
        print("overall ", overall_offset)
        return offset_a, offset_b

    def get_offset_from_amplitude(self, amp):
        if amp == 0.012:
            offset = 0
        elif amp == 0.015:
            offset = self.ppr / 4
        elif ((amp >= 0.02) & (amp <= 0.1) & (abs(amp % 0.005) <= 0.0000001)):
            offset = ((amp - 0.02) * ((self.ppr / 8) / 0.005)) + (self.ppr / 2)
        else:
            offset = 0
        return offset

    # On press apply, edit variables and exit window if inputs are valid
    def on_press_adv_apply(self, event):
        if not self.check_if_in_range(self.arm_length_set.GetValue(), 0, 1):
            print("Error: Arm Length not valid")
        elif not self.check_if_in_range(self.crank_a_amplitude_set.GetValue(), 0.005, float(self.arm_length_set.GetValue())):
            print("Error: Crank A Amplitude not valid")
        elif not self.check_if_in_range(self.crank_b_amplitude_set.GetValue(), 0.005, float(self.arm_length_set.GetValue())):
            print("Error: Crank B Amplitude not valid")
        elif not self.check_if_in_range(self.encoder_absolute_difference_set.GetValue(), -self.parent.ppr, self.parent.ppr):
            print("Error: Absolute Encoder Offset not valid")
        else:
            self.parent.crank_a_amplitude = float(self.crank_a_amplitude_set.GetValue())
            self.parent.crank_b_amplitude = float(self.crank_b_amplitude_set.GetValue())
            self.parent.arm_length = float(self.arm_length_set.GetValue())
            self.parent.encoder_absolute_difference = float(self.encoder_absolute_difference_set.GetValue())
            self.parent.amplitude_setpoint_a, self.parent.amplitude_setpoint_b = self.get_absolute_setpoint_difference(self.parent.crank_a_amplitude, self.parent.crank_b_amplitude)
            self.Destroy()

# UI stuffs
class MyFrame(wx.Frame):    
    # Initialise
    def __init__(self, style=wx.DEFAULT_FRAME_STYLE ^ wx.RESIZE_BORDER):
        #self.style=wx.DEFAULT_FRAME_STYLE ^ wx.RESIZE_BORDER
        self.phase1 = 0.0
        self.phase2 = 0.0
        self.encoder_absolute_difference = 0
        self.amplitude_setpoint_a = 0
        self.amplitude_setpoint_b = 0
        self.arm_length = 0.38
        self.crank_a_amplitude = 0.012
        self.crank_b_amplitude = 0.012

        self.ppr = 192 # Pulses per revolution of the encoders. Defaults to 192 until updated by a return msg from the arduino

        self.init_gui()
        
        self.f1 = "0.0"
        self.f2 = "0.0"
        self.po = "0.0"
        self.lock = False

        self.eth_client = ethernet_client(self.on_msg_recieved)
        self.eth_client.start_reader()

        # Define close logic
        self.Bind(wx.EVT_CLOSE, self.on_close)

    # Setup window layout
    def init_gui(self):
        super().__init__(parent=None, title='Canopy control', size=(717, 200))

        self.advanced_window = None

        self.panel = wx.Panel(self, wx.ID_ANY)      
        self.big_sizer = wx.BoxSizer(wx.HORIZONTAL)  

        self.init_col1()
        self.init_col2()
        self.init_col3()
        self.init_col4()

        self.big_sizer.Add(self.col_1_sizer)
        self.big_sizer.Add(self.col_2_sizer)
        self.big_sizer.Add(self.col_3_sizer)
        self.big_sizer.Add(self.col_4_sizer)

        self.init_col5()
    
        self.panel.SetSizer(self.big_sizer)     
        self.Show()

    # Define Labels column
    def init_col1(self):
        self.col_1_sizer = wx.BoxSizer(wx.VERTICAL) 

        # Title Line
        line1= wx.BoxSizer(wx.HORIZONTAL) 
        l1_label1 = wx.StaticText(self.panel, -1, " ") 
        line1.Add(l1_label1, 0, wx.EXPAND|wx.CENTER|wx.ALL,5)
        self.col_1_sizer.Add(line1)    

        # Frequency 1 set
        line2 = wx.BoxSizer(wx.HORIZONTAL) 
        l2_label1 = wx.StaticText(self.panel, -1, "Motor 1 Frequency") 
        line2.Add(l2_label1, 1, wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,8)
        self.col_1_sizer.Add(line2)

        # Frequency 2 set
        line3 = wx.BoxSizer(wx.HORIZONTAL) 
        l3_label1 = wx.StaticText(self.panel, -1, "Motor 2 Frequency") 
        line3.Add(l3_label1, 1, wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,8)
        self.col_1_sizer.Add(line3)

        # Phase offset
        line4 = wx.BoxSizer(wx.HORIZONTAL) 
        l4_label1 = wx.StaticText(self.panel, -1, "Phase offset") 
        line4.Add(l4_label1, 1, wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,8)
        self.col_1_sizer.Add(line4)

    # Define target input column
    def init_col2(self):
        self.col_2_sizer = wx.BoxSizer(wx.VERTICAL) 

        # Title Line
        l1_label1 = wx.StaticText(self.panel, -1, "Targets") 
        self.col_2_sizer.Add(l1_label1, 0, wx.CENTER|wx.ALL,5)

        # Frequency 1 set
        self.frequency1_set = wx.TextCtrl(self.panel, value = "0", style = wx.TE_READONLY|wx.TE_CENTER) 
        self.col_2_sizer.Add(self.frequency1_set,1,wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,5) 
        self.frequency1_set.Bind(wx.EVT_TEXT, self.f1_event)

        # Frequency 2 set
        self.frequency2_set = wx.TextCtrl(self.panel, value = "0", style = wx.TE_READONLY|wx.TE_CENTER) 
        self.col_2_sizer.Add(self.frequency2_set,1,wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,5) 
        self.frequency2_set.Bind(wx.EVT_TEXT, self.f2_event)

        # Phase offset
        self.phase_offset_set = wx.TextCtrl(self.panel, value = "0", style = wx.TE_READONLY|wx.TE_CENTER) 
        self.col_2_sizer.Add(self.phase_offset_set,1,wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,5) 
        self.phase_offset_set.Bind(wx.EVT_TEXT, self.po_event)

        btns = wx.BoxSizer(wx.HORIZONTAL)
        # Lock btn
        self.lock_btn = wx.Button(self.panel, label='Unlock')
        self.lock_btn.Bind(wx.EVT_BUTTON, self.on_press_lock)
        btns.Add(self.lock_btn, 0, wx.ALL | wx.LEFT, 5)

        # Apply btn
        self.apply_btn = wx.Button(self.panel, label='Apply')
        self.apply_btn.Bind(wx.EVT_BUTTON, self.on_press_apply)
        btns.Add(self.apply_btn, 0, wx.ALL, 5)
        self.col_2_sizer.Add(btns)
    
    # Define increment buttons
    def init_col3(self):
        self.col_3_sizer = wx.BoxSizer(wx.VERTICAL) 

        # Title Line
        l1_label1 = wx.StaticText(self.panel, -1, " ") 
        self.col_3_sizer.Add(l1_label1, 0, wx.EXPAND|wx.CENTER|wx.ALL, 6)

        # increment buttons 
        for btn_params in [['+', "f1_increase", wx.TOP], ['-', "f1_decrease", wx.BOTTOM], ['+', "f2_increase", wx.TOP], ['-', "f2_decrease", wx.BOTTOM], ['+', "po_increase", wx.TOP], ['-', "po_decrease", wx.BOTTOM]]:
            btn = wx.Button(self.panel, label=btn_params[0], size=(35, 16))
            btn.Bind(wx.EVT_BUTTON, lambda evt, temp=btn_params[1]: self.increment_buttons(evt, temp))
            self.col_3_sizer.Add(btn, 0, btn_params[2], 2)

    # Encoder readings column
    def init_col4(self):
        self.col_4_sizer = wx.BoxSizer(wx.VERTICAL) 

        # Title Line
        l1_label1 = wx.StaticText(self.panel, -1, "Encoder readings") 
        self.col_4_sizer.Add(l1_label1, 0, wx.ALL, 5)

        # Frequency 1 set
        self.frequency1_val = wx.TextCtrl(self.panel, value = "0", style = wx.TE_READONLY | wx.TE_CENTER) 
        self.col_4_sizer.Add(self.frequency1_val, 1, wx.EXPAND | wx.ALIGN_LEFT | wx.ALL, 5) 

        # Frequency 2 set
        self.frequency2_val = wx.TextCtrl(self.panel, value = "0", style = wx.TE_READONLY|wx.TE_CENTER) 
        self.col_4_sizer.Add(self.frequency2_val,1,wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,5) 

        # Phase offset
        self.phase_offset_val = wx.TextCtrl(self.panel, value = "0", style = wx.TE_READONLY|wx.TE_CENTER) 
        self.col_4_sizer.Add(self.phase_offset_val,1,wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,5) 
        
        # Btns
        self.col_4_btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        # Stop btn
        self.stop_btn = wx.Button(self.panel, label='Stop',)
        self.stop_btn.Bind(wx.EVT_BUTTON, self.on_press_stop)
        self.col_4_btn_sizer.Add(self.stop_btn, 0, wx.ALL, 5,)

        # Advanced btn
        self.advanced_btn = wx.Button(self.panel, label='Advanced',)
        self.advanced_btn.Bind(wx.EVT_BUTTON, self.on_press_advanced)
        self.col_4_btn_sizer.Add(self.advanced_btn, 0, wx.ALL, 5,)
        self.col_4_sizer.Add(self.col_4_btn_sizer)

    # plot the motion profile
    def generate_profile(self, target_frequency_a, target_frequency_b, initial_phase_offset):
        x = np.zeros(2000)
        y = np.zeros(2000)

        for theta in range(0, 2000):
            phase_a = (theta * math.pi / 100) * target_frequency_a
            phase_b = ((theta * math.pi / 100) * target_frequency_b) + initial_phase_offset
            Ba = math.asin(-(self.crank_a_amplitude * math.sin(phase_a)) / self.arm_length)
            Bb = math.asin(-(self.crank_b_amplitude * math.sin(phase_b)) / self.arm_length)
            x[theta] = self.crank_a_amplitude * math.cos(phase_a) + self.arm_length * math.cos(Ba)
            y[theta] = self.crank_b_amplitude * math.cos(phase_b) + self.arm_length * math.cos(Bb)

        return x, y
    
    def draw_motion_profile(self, f1, f2, po):
        # Generate some Data.
        x_data, y_data = self.generate_profile(f1, f2, po)

        # Requires data as a list of (x, y) pairs:
        xy_data = list(zip(x_data, y_data))

        # Create spline.
        line = PolySpline(
            xy_data,
            colour = wx.Colour(128, 128, 0),   # Color: olive
            width = 3,
        )

        ph_a = float(self.phase1) * (2 * math.pi / self.ppr)
        ph_b = float(self.phase2) * (2 * math.pi / self.ppr)
        Ba = math.asin(-(self.crank_a_amplitude * math.sin(ph_a)) / self.arm_length)
        Bb = math.asin(-(self.crank_b_amplitude * math.sin(ph_b)) / self.arm_length)
        pt_x = self.crank_a_amplitude * math.cos(ph_a) + self.arm_length * math.cos(Ba)
        pt_y = self.crank_b_amplitude * math.cos(ph_b) + self.arm_length * math.cos(Bb)
        
        xborder = (x_data.max() - x_data.min()) / 20
        yborder = (y_data.max() - y_data.min()) / 20
        if xborder > 0:
            xmin = x_data.min() - xborder
            xmax = x_data.max() + xborder
        else:
            xmin = x_data.min() - 0.05
            xmax = x_data.max() + 0.05
        if yborder > 0:
            ymin = y_data.min() - yborder
            ymax = y_data.max() + yborder
        else:
            ymin = y_data.min() - 0.1
            ymax = y_data.max() + 0.1

        point = (pt_x, pt_y)
        marker = PolyMarker(point, marker='circle', size=1)

        return PlotGraphics([line, marker]), xmin, xmax, ymin, ymax
    
    def update_motion_profile(self, f1, f2, po):
        profile, xmin, xmax, ymin, ymax = self.draw_motion_profile(f1, f2, po)

        if ((xmax - xmin) > (ymax - ymin)):
            ycenter = (ymax + ymin) / 2
            ymax = ycenter + ((xmax - xmin) / 2)
            ymin = ycenter - ((xmax - xmin) / 2)
        else:
            xcenter = (xmax + xmin) / 2
            xmax = xcenter + ((ymax - ymin) / 2)
            xmin = xcenter - ((ymax - ymin) / 2)

        self.canvas.Draw(profile, xAxis=(xmin, xmax), yAxis=(ymin, ymax))

    def init_col5(self):
        # Plot the spline
        self.canvas = PlotCanvas(self.panel)
        self.canvas.enableGrid = False
        self.canvas.ySpec = 'none'
        self.canvas.xSpec = 'none'
        self.canvas.SetBackgroundColour(self.panel.GetBackgroundColour())

        self.update_motion_profile(0, 0, 0)

        self.big_sizer.Add(self.canvas, proportion=1, flag=wx.EXPAND)

    # Get value of static text for phase offset
    def po_event(self, event):
        self.po = event.GetString()
        if self.check_if_in_range(self.po, 0, 360):
            self.update_motion_profile(float(self.f1), float(self.f2), float(self.po) * math.pi / 180)

    # Get value of static text for motor 1 frequency
    def f1_event(self, event):
        self.f1 = event.GetString()
        if self.check_if_in_range(self.f1, 0, 3):
            self.update_motion_profile(float(self.f1), float(self.f2), float(self.po) * math.pi / 180)

    # Get value of static text for motor 2 frequency
    def f2_event(self, event):
        self.f2 = event.GetString()
        if self.check_if_in_range(self.f2, 0, 3):
            self.update_motion_profile(float(self.f1), float(self.f2), float(self.po)*math.pi/180)

    # Apply button logic
    def on_press_apply(self, event):
        # If all inputs are valid send string of commands to arduino
        if (self.check_if_in_range(self.f1, 0, 3) & self.check_if_in_range(self.f2, 0, 3) & self.check_if_in_range(self.po, 0, 360)):
            gui_msg = "gui," + self.f1 + ',' + self.f2 + ',' + self.po + '\n'
            self.eth_client.send_msg(gui_msg)

    # Stop button logic
    def on_press_stop(self, event):
        # Send stop command to arduino
        stop_msg = "x\n"
        self.eth_client.send_msg(stop_msg)

        self.frequency1_set.SetValue("0")
        self.frequency2_set.SetValue("0")
        self.phase_offset_set.SetValue("0")

        self.frequency1_val.SetValue("0")
        self.frequency2_val.SetValue("0")
        self.phase_offset_val.SetValue("0")
        self.phase1 = 0
        self.phase2 = 0

        self.update_motion_profile(float(self.f1), float(self.f2), float(self.po) * math.pi / 180)

    def on_press_advanced(self, event):
        if not self.advanced_window:
            self.advanced_window = AdvancedWindow(self)
        self.advanced_window.Show()

    # Lock button logic
    def on_press_lock(self, event):
        # Change editability of target inputs
        self.lock = not self.lock
        self.frequency1_set.SetEditable(self.lock)
        self.frequency2_set.SetEditable(self.lock)
        self.phase_offset_set.SetEditable(self.lock)

        # Change button name
        if (self.lock == False):
            self.lock_btn.SetLabel("Unlock")
        else:
            self.lock_btn.SetLabel("Lock")

    # Check is string is a valid input
    def check_if_in_range(self, val, min, max):
        if (not val.replace(".", "").isnumeric()):
            return 0
        elif (float(val) < min) | (float(val) > max):
            return 0
        return 1

    # Callback for incremental buttons
    def increment_buttons(self, event, label):
        if label == "f1_increase":
            if self.check_if_in_range(str(float(self.f1)+0.1), 0, 3):
                self.f1 = str(round(float(self.f1)+0.1, 2))
                self.update_motion_profile(float(self.f1), float(self.f2), float(self.po)*math.pi/180)
        elif label == "f1_decrease":
            if self.check_if_in_range(str(float(self.f1)-0.1), 0, 3):
                self.f1 = str(round(float(self.f1)-0.1, 2))
                self.update_motion_profile(float(self.f1), float(self.f2), float(self.po)*math.pi/180)
        elif label == "f2_increase":
            if self.check_if_in_range(str(float(self.f2)+0.1), 0, 3):
                self.f2 = str(round(float(self.f2)+0.1, 2))
                self.update_motion_profile(float(self.f1), float(self.f2), float(self.po)*math.pi/180)
        elif label == "f2_decrease":
            if self.check_if_in_range(str(float(self.f2)-0.1), 0, 3):
                self.f2 = str(round(float(self.f2)-0.1, 2))
                self.update_motion_profile(float(self.f1), float(self.f2), float(self.po)*math.pi/180)
        elif label == "po_increase":
            if self.check_if_in_range(str(float(self.po)+15), 0, 360):
                self.po = str(round(float(self.po)+15, 1))
                self.update_motion_profile(float(self.f1), float(self.f2), float(self.po)*math.pi/180)
        elif label == "po_decrease":
            if self.check_if_in_range(str(float(self.po)-15), 0, 360):
                self.po = str(round(float(self.po)-15, 1))
                self.update_motion_profile(float(self.f1), float(self.f2), float(self.po)*math.pi/180)
        
        self.frequency1_set.SetValue(self.f1)
        self.frequency2_set.SetValue(self.f2)
        self.phase_offset_set.SetValue(self.po)
    
    def on_msg_recieved(self, msg):
        msgs = msg.split(',')
        if (len(msgs) == 6):
            self.frequency1_val.SetValue(msgs[0])
            self.frequency2_val.SetValue(msgs[1])
            self.phase_offset_val.SetValue(msgs[2])
            self.ppr = int(msgs[5])
            self.phase1 = str((float(msgs[3]) + self.amplitude_setpoint_a) % self.ppr)
            self.phase2 = str((float(msgs[4]) + self.amplitude_setpoint_b) % self.ppr)

            self.update_motion_profile(float(self.f1), float(self.f2), float(self.po) * math.pi / 180)
        else:
            print(msg, sep="", end="", flush=True)

    # Callback for closing the window
    def on_close(self, evt):
        self.eth_client.close()
        evt.Skip()    

if __name__ == '__main__':
    app = wx.App()
    frame = MyFrame()
    frame.SetMaxSize(wx.Size(767, 250))
    frame.SetMinSize(wx.Size(767, 250))
    app.MainLoop()