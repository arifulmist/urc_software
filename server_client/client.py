import socketio
SERVER_URL = 'http://ipaddress:5000'


sio = socketio.Client()

# This creates a Socket.IO client instance so you can connect, listen, and send messages

@sio.event
def connect():
    print("connected to the server,hurray!!!")


@sio.on('recive_msg')#recive_msg nam er function ei jabe 
def onmsg(data):
    print(f"Received mesage :{data}")
    '''
    This listens for incoming messages from the server using the event name 'receive_message'.

    When a message is broadcasted from the server (using emit('receive_message', ...)), this function will print it.
    '''


sio.connect(SERVER_URL)
#Connects the client to the server via the IP and port you provided.

while True:
    msg=input("enter the send message: ")
    sio.emit('send_message',msg)

    '''
    Keeps asking the user to type a message (input()).

    Each message is sent to the server using the event name 'send_message'.

    The server then broadcasts it back to all connected clients (including this one).
    '''
