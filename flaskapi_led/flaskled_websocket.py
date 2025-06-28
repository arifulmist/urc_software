from flask import Flask,render_template_string
from flask_socketio import SocketIO

app=Flask(__name__)
socketio=SocketIO(app,cors_allowed_origins="*")
html="""
<!DOCTYPE html>
<html>
<head><title>WebSocket LED</title></head>
<body>
  <h2>WebSocket LED Server</h2>
  <p>ESP32 will send values and server will print them.</p>
</body>
</html>
"""
@app.route('/')
def index():
    return render_template_string(html);


@socketio.on('connect')
def onconnect():
    print("Esp32 connected via webSocket")


@socketio.on('message')
def handel_msg(msg):
    print(f"Received: {msg}")
    try :
        val=int(msg)
        if val>1500:
            print("Led ON")
        else :
            print("Led OFF")
    except:
        print("Invalide number received")



if __name__== '__main__':
   socketio.run(app,host='0.0.0.0',port=5000)