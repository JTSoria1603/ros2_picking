import threading
import time
import rclpy
from clients.ros_client import ROSClient

def register_socketio_events(socketio):
    @socketio.on('connect')
    def handle_connect():
        print("Client connected.")

def ros_polling_loop(socketio):
    rclpy.init(args=None)
    ros = ROSClient()

    while True:
        emergency = ros.is_emergency_pressed()
        door_closed = ros.is_door_closed()
        stack_light = int(ros.get_stack_light())
        print(f"Emergency: {emergency}, Door Closed: {door_closed}, Stack Light: {stack_light}")
        socketio.emit('status_update', {
            'emergency': emergency,
            'door_closed': door_closed,
            'stack_light': stack_light
        })

        time.sleep(1)

def start_background_ros_thread(socketio):
    thread = threading.Thread(target=ros_polling_loop, args=(socketio,))
    thread.daemon = True
    thread.start()