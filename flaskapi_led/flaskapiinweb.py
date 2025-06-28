from flask import Flask, request, jsonify, render_template_string, redirect, url_for

app=Flask(__name__)

led_status="OFF"
html_page="""
<!DOCTYPE html>
<html>
<head>
<title>ESp32 Led Controle</title>

</head>
<body style="text-align: center; padding-top: 50px; font-family: Arial;">
<h1>ESP32 LED Control</h1>
 <p>Current LED Status: <strong>{{ status }}</strong></p>
  <form method="POST" action="/led">
    <button name="action" value="ON" style="padding: 10px 30px; font-size: 20px; background:#fff">Turn ON</button>
    <button name="action" value="OFF" style="padding: 10px 30px; font-size: 20px;">Turn OFF</button>
  </form>
</body>
</html>
"""
@app.route('/', methods=['GET'])
def index():
    return render_template_string(html_page, status=led_status)


@app.route('/led', methods=['GET', 'POST'])
def led_control():
    global led_status

    if request.method == 'POST':
        if request.form.get("action"):
            # Came from web form
            action = request.form.get("action")
            led_status = action
            print(f"LED set to: {led_status}")
            return redirect(url_for('index'))  # ✅ Redirect back to web page

        elif request.is_json:
            data = request.get_json()
            action = data.get("action")
            if action in ["ON", "OFF"]:
                led_status = action
                print(f"LED set to: {led_status}")
                return jsonify({"status": "updated", "action": led_status})
            return jsonify({"error": "Invalid action"}), 400

    # GET request returns current LED state (for ESP32)
    return jsonify({"status": "ok", "action": led_status})


if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5000)