import sys
import signal

from threading import Thread

from .actions import ActionRegistry
from .backends import BluetoothBackend, HidrawBackend
from .config import load_options
from .daemon import Daemon
from .eventloop import EventLoop
from .exceptions import BackendError
from ds4drv import DS4Controller,SigintHandler
import rospy
import subprocess
from ds4drv_msgs.msg import DS4_Indication, DS4_ConnectionStatus


def create_controller_thread(index, controller_options, dynamic=False):
    controller = DS4Controller(index, controller_options, dynamic=dynamic)

    thread = Thread(target=controller.run)
    thread.controller = controller
    thread.start()

    return thread

class DS4DRV_Node(object):
    def __init__(self):
    
        self.threads = []
        self.thread_dict = dict()

        self.sigint_handler = SigintHandler(self.threads)
        signal.signal(signal.SIGINT, self.sigint_handler)
        
        self._pub = rospy.Publisher('/joy/connection_status',DS4_ConnectionStatus,queue_size=10) 
        self._t1 = rospy.Timer(rospy.Duration(0.5),self._poll_connection_status)
        
        self._run()

    def _run(self):
        try:
            options = load_options()
        except ValueError as err:
            Daemon.exit("Failed to parse options: {0}", err)

        if options.hidraw:
            backend = HidrawBackend(Daemon.logger)
        else:
            backend = BluetoothBackend(Daemon.logger)

        try:
            backend.setup()
        except BackendError as err:
            Daemon.exit(err)

        if options.daemon:
            Daemon.fork(options.daemon_log, options.daemon_pid)
        
        for index, controller_options in enumerate(options.controllers):
            thread = create_controller_thread(index + 1, controller_options)
            self.threads.append(thread)
            

        for device in backend.devices:
            connected_devices = []
            for thread in self.threads:
                # Controller has received a fatal error, exit
                if thread.controller.error:
                    self._t1.shutdown()
                    self._pub.unregister()
                    sys.exit(1)

                if thread.controller.device:
                    connected_devices.append(thread.controller.device.device_addr)
                    

                # Clean up dynamic threads
                if not thread.is_alive():
                    self.threads.remove(thread)

            if device.device_addr in connected_devices:
                backend.logger.warning("Ignoring already connected device: {0}",
                                       device.device_addr)
                continue

            for thread in filter(lambda t: not t.controller.device, self.threads):
                break
            else:
                thread = create_controller_thread(len(self.threads) + 1,
                                                  options.default_controller,
                                                  dynamic=True)
                self.threads.append(thread)

            thread.controller.setup_device(device)
            bashCommand = "xinput list | grep -Eo 'Wireless Controller Touchpad\s*id\=[0-9]{1,2}' | grep -Eo '[0-9]{1,2}'"

            process = subprocess.Popen(bashCommand, stdout=subprocess.PIPE, shell=True)
            output, error = process.communicate()
            if not error:
                bashCommand = "xinput disable %s"%output
                process = subprocess.Popen(bashCommand, stdout=subprocess.PIPE, shell=True)
                output, error = process.communicate()
            
            thread.controller.device.set_led(255,0,0)
            idx = thread.controller.index
            print idx
            self.thread_dict[idx]=thread
            rospy.Publisher
            rospy.Subscriber('/joy/indication', DS4_Indication, self._indicate)
        
    def _indicate(self,msg):
        self.thread_dict[msg.index].controller.device.control(msg.big_rumble, msg.small_rumble,
                                                              msg.led_red, msg.led_green, msg.led_blue,
                                                              msg.flash_on, msg.flash_off)
                                                              
    def _poll_connection_status(self,event):
        for index, thread in self.thread_dict.iteritems():
            msg = DS4_ConnectionStatus()
            msg.index = index
            msg.reports_per_second = thread.controller.actions[2].rps
            if (thread.controller.device):
                msg.connected = True
            else:
                msg.connected = False
            
            self._pub.publish(msg)
            
        
        
        
            
            
            
        

