#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from flask import Flask, jsonify
from flask_cors import CORS
import subprocess
import time

app = Flask(__name__)
CORS(app)

gazebo_process = None
api_process = None

WORLD_PATH = "/home/ssafy/gazebo_agv_ws/src/agv_sim/worlds/agv_factory_final_box_agv_v2.world"
API_MOVE_SCRIPT = "/home/ssafy/gazebo_agv_ws/src/agv_sim/scripts/move_agv_from_api_server.py"


@app.route("/open-gazebo", methods=["POST", "GET"])
def open_gazebo():
    global gazebo_process, api_process

    try:
        if gazebo_process is None or gazebo_process.poll() is not None:
            gazebo_process = subprocess.Popen([
                "bash",
                "-lc",
                f"source /opt/ros/humble/setup.bash && gazebo --verbose {WORLD_PATH}"
            ])

            time.sleep(5)

        if api_process is None or api_process.poll() is not None:
            api_process = subprocess.Popen([
                "bash",
                "-lc",
                f"source /opt/ros/humble/setup.bash && python3 {API_MOVE_SCRIPT}"
            ])

        return jsonify({
            "success": True,
            "message": "Gazebo simulation started"
        })

    except Exception as e:
        return jsonify({
            "success": False,
            "message": str(e)
        }), 500


@app.route("/stop-gazebo", methods=["POST", "GET"])
def stop_gazebo():
    global gazebo_process, api_process

    if api_process is not None and api_process.poll() is None:
        api_process.terminate()
        api_process = None

    if gazebo_process is not None and gazebo_process.poll() is None:
        gazebo_process.terminate()
        gazebo_process = None

    return jsonify({
        "success": True,
        "message": "Gazebo simulation stopped"
    })


@app.route("/health", methods=["GET"])
def health():
    return jsonify({
        "success": True,
        "message": "gazebo_launcher is running"
    })


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001)