import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ttkbootstrap as ttk

class App(Node):
    def __init__(self):
        super().__init__('joint_tester_node')
        self.jointNames_ = ["LF_HAA", "LF_HFE", "LF_KFE",
                             "LH_HAA", "LH_HFE", "LH_KFE",
                             "RF_HAA", "RF_HFE", "RF_KFE", 
                             "RH_HAA", "RH_HFE", "RH_KFE"]
        self.commandInterface_ = ["position","velocity","effort"]
        self.publisher_ = self.create_publisher(String, '/test_tools/motor_tester/command', 10)

        self.window_ = ttk.Window(themename='darkly', title='Joint Controller', size=(1200, 800))
        self.window_.wm_attributes('-topmost', 1)  # 窗口置顶
        self.createUI()

    def sendCommand(self, joint, interface, value):
        msg = String()
        msg.data = joint  + " "  + interface  + " "  + str(value)
        self.publisher_.publish(msg) 

    def createUI(self):
        frame = ttk.Frame(self.window_, padding=10)
        frame.pack(fill="both", expand=True)

        # 表头：空白接口
        ttk.Label(frame, text="Joint", anchor="center").grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        for j, iface in enumerate(self.commandInterface_, start=1):
            ttk.Label(frame, text=iface, anchor="center").grid(row=0, column=j, padx=5, pady=5, sticky="nsew")

        # 表体：关节行  可编辑 Entry
        for i, joint in enumerate(self.jointNames_, start=1):
            ttk.Label(frame, text=joint, anchor="center").grid(row=i, column=0, padx=5, pady=3, sticky="nsew")
            for j, iface in enumerate(self.commandInterface_, start=1):
                var = ttk.StringVar()
                entry = ttk.Entry(frame, textvariable=var, width=10)
                entry.grid(row=i, column=j, padx=5, pady=3, sticky="nsew")
                # 绑定失焦与回车事件，发送命令
                entry.bind("<FocusOut>", lambda e, jt=joint, itf=iface, v=var: self.sendCommand(jt, itf, v.get()))
                entry.bind("<Return>",  lambda e, jt=joint, itf=iface, v=var: self.sendCommand(jt, itf, v.get()))

        # 列宽拉伸
        for col in range(len(self.commandInterface_) +1):
            frame.columnconfigure(col, weight=1)
        for row in range(len(self.jointNames_) + 1):
            frame.rowconfigure(row, weight=1)

    def run(self):
        self.window_.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = App()
    node.run()
    rclpy.shutdown()