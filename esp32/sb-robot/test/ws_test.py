import websocket
import json

def test_websocket_server():
    ws_url = f"ws://192.168.1.77:80/ws"
    ws = websocket.create_connection(ws_url)
    response = ws.recv()
    

    # Authenticate
    ws.send(f"token:your_secret_token")
    response = ws.recv()
    print(f"Server response: {response}")

    if response == "Authenticated successfully":
        print("Authentication successful.")
        # Send commands
        ws.send("init")
        response = ws.recv()
        print(f"Server response: {response}")

        ws.send("reset")
        response = ws.recv()
        print(f"Server response: {response}")

        ws.send("state")
        response = ws.recv()
        print(f"Server response: {response}")

        ws.send("ping")
        response = ws.recv()
        print(f"Server response: {response}")
    else:
        print("Authentication failed. Response:", response)

    ws.close()

if __name__ == "__main__":
    test_websocket_server()
