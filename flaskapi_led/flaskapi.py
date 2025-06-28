from flask import Flask, request, jsonify

app = Flask(__name__)

# This variable controls the LED state
led_status = "OFF"

@app.route('/led', methods=['GET', 'POST'])
def led_control():
    global led_status

    if request.method == 'POST':
        data = request.get_json()
        action = data.get("action")
        if action in ["ON", "OFF"]:
            led_status = action
            print(f"LED set to: {led_status}")
            return jsonify({"status": "updated", "action": led_status})
        else:
            return jsonify({"error": "Invalid action"}), 400

    # If GET request — just return the current LED state
    return jsonify({"status": "ok", "action": led_status})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
