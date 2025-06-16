from flask import Flask,render_template
from flask_socketio import SocketIO,emit

'''Flask: Web framework for Python.

render_template: Used to serve HTML files (you’re not using it here yet).

SocketIO: Adds WebSocket (real-time communication) support to Flask.

emit: Sends messages to connected clients.'''


app=Flask(__name__)#This creates a Flask app instance — the base of your web server.


socketio=SocketIO(app,cors_allowed_origins="*")
#This wraps your Flask app with SocketIO, allowing real-time WebSocket communication.
#cors_allowed_origins="*": Allows connections from any origin (any IP, useful for local network devices).

@app.route('/')
def index():
    return "Websocket server is running ---"
    #If someone visits your server's IP/port (like http://192.168.0.101:5000), this function will return plain text as a response




@socketio.on('send_message')
def handel_msg(data):
    print(f"Recived:{data}")
    emit('recive_msg',data,broadcast=True)
    
    '''
     Listens for a WebSocket event named 'send_message'

    data is the message sent from a client

    print(...): Shows the message in server logs

    emit(..., broadcast=True): Sends the same message back to all connected clients, including other devices — real-time!
    '''

if __name__=='__main__':
    socketio.run(app,host='0.0.0.0',port=5000)



