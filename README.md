# Flask + ROS 2 Integration

This project demonstrates how to integrate a Flask web server with ROS 2 using Python. It allows communication between HTTP clients (like browsers or `curl`) and ROS 2 nodes via GET and POST requests.

---

## ✅ Features

- **GET requests** to publish messages to a ROS 2 topic  
- **POST requests** to call ROS 2 services and return structured JSON responses

---

## 🧠 Tech Stack

- **Flask** – HTTP API framework  
- **rclpy** – Python client library for ROS 2  
- **ROS 2** – Robot Operating System 2 (Humble/Galactic/Foxy supported)

---

## 📦 Installation

### 1. Source your ROS 2 environment:

source /opt/ros/humble/setup.bash

### 2. Install Flask (if not already installed):

pip install flask

---

## 🚀 Running the App

python3 app.py

---

## 🌐 API Endpoints

### 🔹 1. Publish a message to a ROS 2 topic

- **Endpoint:** GET /send_message/<message>  
- **Description:** Publishes a message to the `/my_topic` ROS topic.

**Example:**

curl http://localhost:5000/send_message/hello_world

**Flask Response:**

Sent message: hello_world

**ROS Topic Output (via ros2 topic echo):**

data: "hello_world"

---

### 🔹 2. Trigger a ROS 2 service

- **Endpoint:** POST /trigger  
- **Description:** Calls the ROS 2 service `/trigger_me` and returns the result.

**Example:**

curl -X POST http://localhost:5000/trigger

**JSON Response:**

{
  "success": true,
  "message": "you triggered me"
}

---

## 📝 Notes

- Use **GET** requests for retrieving or sending simple data (e.g., topic messages).
- Use **POST** requests for triggering services or operations that change state.
- `rclpy.spin_until_future_complete()` is used to wait for the service result synchronously.

---

## 🔧 Possible Extensions

- Add additional routes for other services or topics  
- Implement logging and error handling  
- Add authentication headers for secure production deployments  
- Connect with a front-end dashboard
