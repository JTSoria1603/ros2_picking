from flask import Flask, render_template
from flask_socketio import SocketIO
from sockets import register_socketio_events, start_background_ros_thread

app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading')

register_socketio_events(socketio)

@app.route("/")
def index():
    return render_template("index.html")

if __name__ == "__main__":
    start_background_ros_thread(socketio)
    socketio.run(app, host="127.0.0.1", port=8082,allow_unsafe_werkzeug=True)