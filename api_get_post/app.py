from flask import Flask,request,jsonify;

app=Flask(__name__)

@app.route('/api/message',methods=['GET'])
def message():
    return jsonify({"message":"Hello from get"})



@app.route('/api/message',methods=['POST'])
def post():
    data=request.json
    name=data.get('name','Guest')
    return jsonify({"message":f"Hello ,{name} from POSt"})





if __name__=='__main__':
    app.run(debug=True)