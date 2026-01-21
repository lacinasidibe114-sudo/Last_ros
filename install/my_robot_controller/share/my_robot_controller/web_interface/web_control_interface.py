#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from flask import Flask, render_template_string, request, jsonify
from flask_cors import CORS
import threading
import numpy as np
import base64
from io import BytesIO
import json
import time
import math
import subprocess
import os
import socket

# Template HTML avec SLAM intégré
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🏥 Système de Navigation Hospitalière SLAM</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%);
            min-height: 100vh;
            padding: 20px;
            color: white;
        }

        .dashboard {
            display: grid;
            grid-template-columns: 350px 1fr 350px;
            grid-template-rows: auto 1fr;
            gap: 20px;
            max-width: 1800px;
            margin: 0 auto;
            height: calc(100vh - 40px);
        }

        .panel {
            background: rgba(255, 255, 255, 0.95);
            border-radius: 20px;
            padding: 25px;
            box-shadow: 0 10px 40px rgba(0, 0, 0, 0.3);
            color: #333;
            overflow: hidden;
        }

        .header {
            grid-column: 1 / -1;
            text-align: center;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 20px;
            margin-bottom: 0;
        }

        .header h1 {
            font-size: 2em;
            margin-bottom: 10px;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 15px;
        }

        .status-bar {
            display: flex;
            gap: 15px;
            justify-content: center;
            margin-top: 10px;
            flex-wrap: wrap;
        }

        .status-item {
            background: rgba(255, 255, 255, 0.2);
            padding: 8px 15px;
            border-radius: 15px;
            font-weight: bold;
            backdrop-filter: blur(10px);
            font-size: 0.9em;
        }

        .status-item.active {
            background: #4ade80;
            animation: pulse 2s ease-in-out infinite;
        }

        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.7; }
        }

        .section-title {
            font-size: 1.3em;
            color: #667eea;
            margin-bottom: 15px;
            display: flex;
            align-items: center;
            gap: 8px;
            border-bottom: 2px solid #667eea;
            padding-bottom: 8px;
        }

        .control-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 8px;
            margin-bottom: 15px;
        }

        .control-btn {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            border: none;
            color: white;
            font-size: 1.5em;
            padding: 15px;
            border-radius: 12px;
            cursor: pointer;
            transition: all 0.3s ease;
            box-shadow: 0 5px 15px rgba(102, 126, 234, 0.4);
        }

        .control-btn:hover {
            transform: translateY(-3px);
            box-shadow: 0 8px 20px rgba(102, 126, 234, 0.6);
        }

        .control-btn.active {
            background: linear-gradient(135deg, #4ade80 0%, #22c55e 100%);
            box-shadow: 0 0 30px rgba(74, 222, 128, 0.6);
        }

        .stop-btn {
            background: linear-gradient(135deg, #ef4444 0%, #dc2626 100%);
            font-size: 1.1em;
            font-weight: bold;
        }

        .speed-control {
            margin: 10px 0;
        }

        .speed-control label {
            display: flex;
            justify-content: space-between;
            margin-bottom: 5px;
            font-weight: 600;
            color: #555;
            font-size: 0.9em;
        }

        .speed-control input[type="range"] {
            width: 100%;
            height: 5px;
            border-radius: 5px;
            background: #ddd;
            outline: none;
        }

        .speed-control input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 18px;
            height: 18px;
            border-radius: 50%;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            cursor: pointer;
        }

        #map-container {
            background: #1a1a2e;
            border-radius: 15px;
            padding: 10px;
            height: calc(100% - 120px);
            position: relative;
            overflow: hidden;
            border: 3px solid #667eea;
        }

        #map-canvas {
            width: 100%;
            height: 100%;
            background: #0f0f1e;
            border-radius: 8px;
            cursor: crosshair;
        }

        .map-overlay {
            position: absolute;
            top: 15px;
            left: 15px;
            background: rgba(0, 0, 0, 0.8);
            padding: 12px;
            border-radius: 8px;
            font-size: 0.85em;
            backdrop-filter: blur(10px);
            z-index: 10;
        }

        .map-controls {
            position: absolute;
            top: 15px;
            right: 15px;
            display: flex;
            flex-direction: column;
            gap: 8px;
            z-index: 10;
        }

        .map-btn {
            background: rgba(102, 126, 234, 0.9);
            border: none;
            color: white;
            padding: 10px 14px;
            border-radius: 8px;
            cursor: pointer;
            font-size: 1.1em;
            transition: all 0.3s ease;
            backdrop-filter: blur(10px);
        }

        .map-btn:hover {
            background: rgba(102, 126, 234, 1);
            transform: scale(1.1);
        }

        .metrics-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 10px;
            margin-bottom: 15px;
        }

        .metric-card {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            padding: 15px;
            border-radius: 12px;
            color: white;
            text-align: center;
        }

        .metric-value {
            font-size: 1.5em;
            font-weight: bold;
            margin: 8px 0;
        }

        .metric-label {
            font-size: 0.85em;
            opacity: 0.9;
        }

        .laser-visualization {
            background: #0f0f1e;
            border-radius: 12px;
            padding: 10px;
            height: 180px;
            position: relative;
            border: 2px solid #667eea;
            margin-top: 10px;
        }

        #laser-canvas {
            width: 100%;
            height: 100%;
        }

        .waypoint-list {
            max-height: 200px;
            overflow-y: auto;
            background: #f8f9fa;
            border-radius: 8px;
            padding: 8px;
            margin-bottom: 15px;
        }

        .waypoint-item {
            background: white;
            padding: 12px;
            margin-bottom: 8px;
            border-radius: 8px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-left: 4px solid #667eea;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
            font-size: 0.9em;
        }

        .waypoint-item.active {
            border-left-color: #4ade80;
            background: #f0fdf4;
        }

        .waypoint-info {
            flex: 1;
        }

        .waypoint-coords {
            font-size: 0.8em;
            color: #666;
            margin-top: 3px;
        }

        .waypoint-actions {
            display: flex;
            gap: 5px;
        }

        .action-btn {
            padding: 6px 10px;
            border: none;
            border-radius: 6px;
            cursor: pointer;
            font-size: 0.85em;
            transition: all 0.3s ease;
        }

        .action-btn.go {
            background: #4ade80;
            color: white;
        }

        .action-btn.delete {
            background: #ef4444;
            color: white;
        }

        .action-btn:hover {
            transform: scale(1.1);
        }

        .slam-actions {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 8px;
            margin-top: 15px;
        }

        .slam-btn {
            padding: 12px;
            border: none;
            border-radius: 10px;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.3s ease;
            font-size: 0.9em;
        }

        .slam-btn.save {
            background: linear-gradient(135deg, #4ade80 0%, #22c55e 100%);
            color: white;
        }

        .slam-btn.reset {
            background: linear-gradient(135deg, #f59e0b 0%, #d97706 100%);
            color: white;
        }

        .slam-btn.clear {
            background: linear-gradient(135deg, #ef4444 0%, #dc2626 100%);
            color: white;
        }

        .slam-btn.localize {
            background: linear-gradient(135deg, #8b5cf6 0%, #7c3aed 100%);
            color: white;
        }

        .slam-btn:hover {
            transform: translateY(-3px);
            box-shadow: 0 5px 15px rgba(0, 0, 0, 0.3);
        }

        .notification {
            position: fixed;
            top: 20px;
            right: 20px;
            background: white;
            padding: 15px;
            border-radius: 12px;
            box-shadow: 0 10px 30px rgba(0, 0, 0, 0.3);
            min-width: 250px;
            transform: translateX(400px);
            transition: transform 0.3s ease;
            z-index: 1000;
            font-size: 0.9em;
        }

        .notification.show {
            transform: translateX(0);
        }

        .notification.success {
            border-left: 5px solid #4ade80;
        }

        .notification.error {
            border-left: 5px solid #ef4444;
        }

        .notification.info {
            border-left: 5px solid #667eea;
        }

        .stats-row {
            display: flex;
            justify-content: space-around;
            padding: 12px;
            background: #f8f9fa;
            border-radius: 8px;
            margin-top: 12px;
        }

        .stat-item {
            text-align: center;
        }

        .stat-value {
            font-size: 1.2em;
            font-weight: bold;
            color: #667eea;
        }

        .stat-label {
            font-size: 0.8em;
            color: #666;
            margin-top: 3px;
        }

        .mode-selector {
            display: flex;
            gap: 8px;
            margin-bottom: 15px;
        }

        .mode-btn {
            flex: 1;
            padding: 10px;
            border: 2px solid #667eea;
            background: white;
            border-radius: 8px;
            cursor: pointer;
            font-weight: bold;
            color: #667eea;
            transition: all 0.3s ease;
            font-size: 0.9em;
        }

        .mode-btn.active {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
        }

        .mode-btn:hover {
            transform: translateY(-2px);
        }

        .locations-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 10px;
            margin-bottom: 15px;
        }

        .location-btn {
            background: linear-gradient(135deg, #3b82f6 0%, #1d4ed8 100%);
            color: white;
            border: none;
            padding: 12px;
            border-radius: 10px;
            cursor: pointer;
            font-weight: 600;
            transition: all 0.3s ease;
            font-size: 0.85em;
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 5px;
        }

        .location-btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(59, 130, 246, 0.4);
        }

        .location-btn em {
            font-size: 1.2em;
        }

        .location-name {
            font-size: 0.8em;
        }

        .connection-status {
            position: fixed;
            bottom: 20px;
            right: 20px;
            background: rgba(0, 0, 0, 0.8);
            color: white;
            padding: 10px 15px;
            border-radius: 8px;
            font-size: 0.85em;
            backdrop-filter: blur(10px);
            z-index: 1000;
        }

        .connection-status.connected {
            background: rgba(34, 197, 94, 0.9);
        }

        .connection-status.disconnected {
            background: rgba(239, 68, 68, 0.9);
        }

        @media (max-width: 1400px) {
            .dashboard {
                grid-template-columns: 300px 1fr 300px;
            }
        }

        @media (max-width: 1200px) {
            .dashboard {
                grid-template-columns: 1fr;
                grid-template-rows: auto auto auto auto;
                height: auto;
            }
        }

        .loading {
            display: inline-block;
            width: 16px;
            height: 16px;
            border: 2px solid rgba(255,255,255,.3);
            border-radius: 50%;
            border-top-color: white;
            animation: spin 1s ease-in-out infinite;
        }

        @keyframes spin {
            to { transform: rotate(360deg); }
        }

        .control-hint {
            font-size: 0.8em;
            color: #666;
            text-align: center;
            margin-top: 10px;
            font-style: italic;
        }

        .slam-options {
            background: #f8f9fa;
            padding: 12px;
            border-radius: 10px;
            margin-top: 15px;
            font-size: 0.9em;
        }

        .option-item {
            display: flex;
            align-items: center;
            margin-bottom: 8px;
        }

        .option-item:last-child {
            margin-bottom: 0;
        }

        .option-item input {
            margin-right: 8px;
        }
    </style>
</head>
<body>
    <div class="dashboard">
        <!-- Header -->
        <div class="panel header">
            <h1>
                <span>🏥</span>
                Navigation Hospitalière SLAM
                <span>🤖</span>
            </h1>
            <div class="status-bar">
                <div class="status-item active" id="slam-status">● SLAM Actif</div>
                <div class="status-item" id="robot-status">⚡ Robot Prêt</div>
                <div class="status-item" id="map-status">🗺️ Cartographie: 0%</div>
            </div>
        </div>

        <!-- Panneau de Contrôle -->
        <div class="panel">
            <div class="section-title">
                <span>🎮</span>
                Contrôle Manuel
            </div>

            <div class="control-grid">
                <button class="control-btn" data-command="forward_left">↖</button>
                <button class="control-btn" data-command="forward">↑</button>
                <button class="control-btn" data-command="forward_right">↗</button>
                
                <button class="control-btn" data-command="left">←</button>
                <button class="control-btn stop-btn" data-command="stop">STOP</button>
                <button class="control-btn" data-command="right">→</button>
                
                <button class="control-btn" data-command="backward_left">↙</button>
                <button class="control-btn" data-command="backward">↓</button>
                <button class="control-btn" data-command="backward_right">↘</button>
            </div>

            <div class="control-hint">
                💡 Touches: W A S D ou flèches
            </div>

            <div class="speed-control">
                <label>
                    <span>Vitesse Linéaire</span>
                    <span id="linear-value">0.5 m/s</span>
                </label>
                <input type="range" id="linear-speed" min="0.1" max="2.0" step="0.1" value="0.5">
            </div>

            <div class="speed-control">
                <label>
                    <span>Vitesse Angulaire</span>
                    <span id="angular-value">1.0 rad/s</span>
                </label>
                <input type="range" id="angular-speed" min="0.1" max="3.0" step="0.1" value="1.0">
            </div>

            <div class="section-title">
                <span>📊</span>
                Métriques Temps Réel
            </div>

            <div class="metrics-grid">
                <div class="metric-card">
                    <div class="metric-label">Position X</div>
                    <div class="metric-value" id="pos-x">0.00</div>
                </div>
                <div class="metric-card">
                    <div class="metric-label">Position Y</div>
                    <div class="metric-value" id="pos-y">0.00</div>
                </div>
                <div class="metric-card">
                    <div class="metric-label">Vitesse</div>
                    <div class="metric-value" id="velocity">0.00</div>
                </div>
                <div class="metric-card">
                    <div class="metric-label">Rotation</div>
                    <div class="metric-value" id="rotation">0.0°</div>
                </div>
            </div>

            <div class="section-title">
                <span>📡</span>
                Capteur LiDAR
            </div>
            <div class="laser-visualization">
                <canvas id="laser-canvas"></canvas>
            </div>
        </div>

        <!-- Carte SLAM -->
        <div class="panel">
            <div class="section-title">
                <span>🗺️</span>
                Carte SLAM en Direct
            </div>

            <div id="map-container">
                <canvas id="map-canvas"></canvas>
                <div class="map-overlay">
                    <div style="color: #4ade80; font-weight: bold;">🤖 Robot</div>
                    <div style="color: #ef4444;">📍 Waypoints</div>
                    <div style="color: #fbbf24;">🎯 Objectif</div>
                    <div style="margin-top: 8px; padding-top: 8px; border-top: 1px solid #333; font-size: 0.8em;">
                        <div>Distance: <span id="distance-traveled">0.0</span> m</div>
                        <div>Zones: <span id="explored-area">0</span> m²</div>
                    </div>
                </div>
                <div class="map-controls">
                    <button class="map-btn" onclick="zoomIn()" title="Zoom In">+</button>
                    <button class="map-btn" onclick="zoomOut()" title="Zoom Out">−</button>
                    <button class="map-btn" onclick="centerMap()" title="Centrer">⌖</button>
                    <button class="map-btn" onclick="toggleRobotTrail()" title="Trajectoire">↗</button>
                </div>
            </div>

            <div class="slam-actions">
                <button class="slam-btn save" onclick="saveMap()">
                    💾 Sauvegarder
                </button>
                <button class="slam-btn reset" onclick="resetPose()">
                    🔄 Réinitialiser
                </button>
                <button class="slam-btn clear" onclick="clearMap()">
                    🗑️ Effacer
                </button>
                <button class="slam-btn localize" onclick="relocalize()">
                    📍 Relocaliser
                </button>
            </div>

            <div class="stats-row">
                <div class="stat-item">
                    <div class="stat-value" id="waypoints-count">0</div>
                    <div class="stat-label">Waypoints</div>
                </div>
                <div class="stat-item">
                    <div class="stat-value" id="scan-quality">0%</div>
                    <div class="stat-label">Qualité Scan</div>
                </div>
                <div class="stat-item">
                    <div class="stat-value" id="loop-closures">0</div>
                    <div class="stat-label">Loop Closures</div>
                </div>
            </div>
        </div>

        <!-- Waypoints et Navigation -->
        <div class="panel">
            <div class="section-title">
                <span>📍</span>
                Points de Navigation
            </div>

            <div style="margin-bottom: 12px; padding: 12px; background: #e0e7ff; border-radius: 8px; color: #667eea; font-weight: 600; font-size: 0.9em;">
                💡 Cliquez sur la carte pour ajouter des waypoints
            </div>

            <div class="waypoint-list" id="waypoint-list">
                <div style="text-align: center; color: #999; padding: 30px; font-size: 0.9em;">
                    Aucun waypoint défini<br>
                    <small>Cliquez sur la carte pour en ajouter</small>
                </div>
            </div>

            <div class="section-title">
                <span>🏥</span>
                Salles de l'Hôpital
            </div>

            <div class="locations-grid">
                <button class="location-btn" onclick="goToLocation('reception')">
                    <em>🏢</em>
                    <div class="location-name">Réception</div>
                </button>
                <button class="location-btn" onclick="goToLocation('emergency')">
                    <em>🚨</em>
                    <div class="location-name">Urgences</div>
                </button>
                <button class="location-btn" onclick="goToLocation('pharmacy')">
                    <em>💊</em>
                    <div class="location-name">Pharmacie</div>
                </button>
                <button class="location-btn" onclick="goToLocation('lab')">
                    <em>🔬</em>
                    <div class="location-name">Laboratoire</div>
                </button>
                <button class="location-btn" onclick="goToLocation('consult1')">
                    <em>👨‍⚕️</em>
                    <div class="location-name">Consultation 1</div>
                </button>
                <button class="location-btn" onclick="goToLocation('consult2')">
                    <em>👩‍⚕️</em>
                    <div class="location-name">Consultation 2</div>
                </button>
                <button class="location-btn" onclick="goToLocation('consult3')">
                    <em>🧑‍⚕️</em>
                    <div class="location-name">Consultation 3</div>
                </button>
                <button class="location-btn" onclick="goToLocation('entrance')">
                    <em>🚪</em>
                    <div class="location-name">Entrée</div>
                </button>
            </div>

            <div class="section-title">
                <span>⚙️</span>
                Options SLAM
            </div>

            <div class="slam-options">
                <div class="option-item">
                    <input type="checkbox" id="auto-explore">
                    <label for="auto-explore">Exploration Automatique</label>
                </div>
                <div class="option-item">
                    <input type="checkbox" id="avoid-obstacles" checked>
                    <label for="avoid-obstacles">Évitement d'Obstacles</label>
                </div>
                <div class="option-item">
                    <input type="checkbox" id="loop-closure" checked>
                    <label for="loop-closure">Loop Closure Detection</label>
                </div>
            </div>
        </div>
    </div>

    <!-- Notification -->
    <div class="notification" id="notification"></div>
    
    <!-- Connection Status -->
    <div class="connection-status connected" id="connection-status">
        ● Connecté
    </div>

    <script>
        let currentCommand = 'stop';
        let waypoints = [];
        let robotPose = {x: 0, y: 0, theta: 0};
        let mapData = null;
        let mapZoom = 20.0;
        let mapOffset = {x: 0, y: 0};
        let showTrail = true;
        let robotTrail = [];
        let lastMousePos = null;
        let isDragging = false;
        let connectionActive = true;
        let lastUpdateTime = Date.now();

        const mapCanvas = document.getElementById('map-canvas');
        const mapCtx = mapCanvas.getContext('2d');
        const laserCanvas = document.getElementById('laser-canvas');
        const laserCtx = laserCanvas.getContext('2d');

        // Resize canvases
        function resizeCanvases() {
            mapCanvas.width = mapCanvas.offsetWidth;
            mapCanvas.height = mapCanvas.offsetHeight;
            laserCanvas.width = laserCanvas.offsetWidth;
            laserCanvas.height = laserCanvas.offsetHeight;
            drawMap();
        }
        resizeCanvases();
        window.addEventListener('resize', resizeCanvases);

        // Check connection status
        function checkConnection() {
            const now = Date.now();
            const timeSinceUpdate = now - lastUpdateTime;
            const statusElement = document.getElementById('connection-status');
            
            if (timeSinceUpdate > 3000) {
                connectionActive = false;
                statusElement.className = 'connection-status disconnected';
                statusElement.textContent = '● Déconnecté';
            } else {
                connectionActive = true;
                statusElement.className = 'connection-status connected';
                statusElement.textContent = '● Connecté';
            }
        }
        setInterval(checkConnection, 1000);

        // Notifications
        function showNotification(message, type = 'info') {
            const notification = document.getElementById('notification');
            notification.textContent = message;
            notification.className = `notification ${type} show`;
            setTimeout(() => {
                notification.classList.remove('show');
            }, 3000);
        }

        // Control commands
        async function sendCommand(command) {
            try {
                const response = await fetch('/control', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({ command: command })
                });
                const data = await response.json();
                
                if (data.status === 'success') {
                    currentCommand = command;
                    updateActiveButton(command);
                    document.getElementById('robot-status').textContent = 
                        `⚡ ${command.charAt(0).toUpperCase() + command.slice(1)}`;
                    lastUpdateTime = Date.now();
                }
            } catch (error) {
                console.error('Connection error:', error);
                showNotification('Erreur de connexion robot', 'error');
            }
        }

        function updateActiveButton(command) {
            document.querySelectorAll('.control-btn').forEach(btn => {
                btn.classList.remove('active');
            });
            const activeBtn = document.querySelector(`[data-command="${command}"]`);
            if (activeBtn) activeBtn.classList.add('active');
        }

        // Control buttons
        document.querySelectorAll('.control-btn').forEach(button => {
            button.addEventListener('mousedown', () => {
                sendCommand(button.dataset.command);
            });
            
            button.addEventListener('mouseup', () => {
                if (button.dataset.command !== 'stop') {
                    sendCommand('stop');
                }
            });
            
            button.addEventListener('mouseleave', () => {
                if (button.dataset.command !== 'stop') {
                    sendCommand('stop');
                }
            });
        });

        // Speed controls
        document.getElementById('linear-speed').addEventListener('input', function() {
            document.getElementById('linear-value').textContent = this.value + ' m/s';
            updateSpeed();
        });

        document.getElementById('angular-speed').addEventListener('input', function() {
            document.getElementById('angular-value').textContent = this.value + ' rad/s';
            updateSpeed();
        });

        async function updateSpeed() {
            const linear = document.getElementById('linear-speed').value;
            const angular = document.getElementById('angular-speed').value;
            
            try {
                await fetch('/set_speed', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({
                        linear: parseFloat(linear), 
                        angular: parseFloat(angular)
                    })
                });
                lastUpdateTime = Date.now();
            } catch (error) {
                console.error('Error updating speed:', error);
            }
        }

        // Keyboard controls
        const keyMap = {
            'ArrowUp': 'forward', 'ArrowDown': 'backward',
            'ArrowLeft': 'left', 'ArrowRight': 'right',
            'w': 'forward', 's': 'backward', 'a': 'left', 'd': 'right',
            ' ': 'stop', 'Escape': 'stop'
        };

        let activeKeys = new Set();

        document.addEventListener('keydown', (e) => {
            if (keyMap[e.key]) {
                e.preventDefault();
                activeKeys.add(e.key);
                sendCommand(keyMap[e.key]);
            }
        });

        document.addEventListener('keyup', (e) => {
            if (keyMap[e.key]) {
                e.preventDefault();
                activeKeys.delete(e.key);
                if (activeKeys.size === 0) {
                    sendCommand('stop');
                }
            }
        });

        // Map interaction
        mapCanvas.addEventListener('click', (e) => {
            if (isDragging) return;
            
            const rect = mapCanvas.getBoundingClientRect();
            const x = (e.clientX - rect.left - mapCanvas.width/2 + mapOffset.x) / mapZoom;
            const y = (e.clientY - rect.top - mapCanvas.height/2 + mapOffset.y) / mapZoom;
            
            addWaypoint(x, -y);
        });

        mapCanvas.addEventListener('mousedown', (e) => {
            isDragging = true;
            lastMousePos = {x: e.clientX, y: e.clientY};
            mapCanvas.style.cursor = 'grabbing';
        });

        mapCanvas.addEventListener('mousemove', (e) => {
            if (!isDragging || !lastMousePos) return;
            
            mapOffset.x += e.clientX - lastMousePos.x;
            mapOffset.y += e.clientY - lastMousePos.y;
            lastMousePos = {x: e.clientX, y: e.clientY};
            drawMap();
        });

        mapCanvas.addEventListener('mouseup', () => {
            isDragging = false;
            mapCanvas.style.cursor = 'crosshair';
        });

        mapCanvas.addEventListener('mouseleave', () => {
            isDragging = false;
            mapCanvas.style.cursor = 'crosshair';
        });

        async function addWaypoint(x, y) {
            const waypoint = {
                id: Date.now(),
                x: x.toFixed(2),
                y: y.toFixed(2),
                name: `Point ${waypoints.length + 1}`
            };
            waypoints.push(waypoint);
            
            try {
                await fetch('/add_waypoint', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(waypoint)
                });
                updateWaypointList();
                showNotification(`Waypoint ajouté: (${x.toFixed(2)}, ${y.toFixed(2)})`, 'success');
                lastUpdateTime = Date.now();
            } catch (error) {
                showNotification('Erreur ajout waypoint', 'error');
            }
        }

        function updateWaypointList() {
            const list = document.getElementById('waypoint-list');
            document.getElementById('waypoints-count').textContent = waypoints.length;
            
            if (waypoints.length === 0) {
                list.innerHTML = `
                    <div style="text-align: center; color: #999; padding: 30px; font-size: 0.9em;">
                        Aucun waypoint défini<br>
                        <small>Cliquez sur la carte pour en ajouter</small>
                    </div>
                `;
                return;
            }
            
            list.innerHTML = waypoints.map((wp, index) => `
                <div class="waypoint-item">
                    <div class="waypoint-info">
                        <strong>${wp.name}</strong>
                        <div class="waypoint-coords">📍 X: ${wp.x}m, Y: ${wp.y}m</div>
                    </div>
                    <div class="waypoint-actions">
                        <button class="action-btn go" onclick="goToWaypoint(${index})">
                            ➜
                        </button>
                        <button class="action-btn delete" onclick="deleteWaypoint(${index})">
                            ✕
                        </button>
                    </div>
                </div>
            `).join('');
        }

        async function goToWaypoint(index) {
            const wp = waypoints[index];
            showNotification(`Navigation vers ${wp.name}...`, 'info');
            
            try {
                await fetch('/goto_waypoint', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(wp)
                });
                lastUpdateTime = Date.now();
            } catch (error) {
                showNotification('Erreur navigation', 'error');
            }
        }

        function deleteWaypoint(index) {
            waypoints.splice(index, 1);
            updateWaypointList();
            showNotification('Waypoint supprimé', 'info');
        }

        async function goToLocation(location) {
            showNotification(`Navigation vers ${location}...`, 'info');
            
            try {
                const response = await fetch('/goto_location', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({location: location})
                });
                const data = await response.json();
                if (data.status === 'success') {
                    showNotification(`En route vers ${location}`, 'success');
                }
                lastUpdateTime = Date.now();
            } catch (error) {
                showNotification('Erreur navigation', 'error');
            }
        }

        // SLAM actions
        async function saveMap() {
            showNotification('Sauvegarde de la carte...', 'info');
            try {
                const response = await fetch('/save_map', {method: 'POST'});
                const data = await response.json();
                if (data.status === 'success') {
                    showNotification('Carte sauvegardée avec succès!', 'success');
                } else {
                    showNotification('Erreur: ' + data.message, 'error');
                }
                lastUpdateTime = Date.now();
            } catch (error) {
                showNotification('Erreur sauvegarde', 'error');
            }
        }

        async function resetPose() {
            showNotification('Réinitialisation de la pose...', 'info');
            try {
                await fetch('/reset_pose', {method: 'POST'});
                showNotification('Pose réinitialisée', 'success');
                lastUpdateTime = Date.now();
            } catch (error) {
                showNotification('Erreur réinitialisation', 'error');
            }
        }

        async function clearMap() {
            if (confirm('Effacer toute la carte?')) {
                showNotification('Effacement de la carte...', 'info');
                try {
                    await fetch('/clear_map', {method: 'POST'});
                    waypoints = [];
                    updateWaypointList();
                    showNotification('Carte effacée', 'success');
                    lastUpdateTime = Date.now();
                } catch (error) {
                    showNotification('Erreur effacement', 'error');
                }
            }
        }

        async function relocalize() {
            showNotification('Relocalisation en cours...', 'info');
            try {
                await fetch('/relocalize', {method: 'POST'});
                showNotification('Relocalisation terminée', 'success');
                lastUpdateTime = Date.now();
            } catch (error) {
                showNotification('Erreur relocalisation', 'error');
            }
        }

        // Map controls
        function zoomIn() {
            mapZoom *= 1.2;
            drawMap();
        }

        function zoomOut() {
            mapZoom /= 1.2;
            drawMap();
        }

        function centerMap() {
            mapOffset = {x: 0, y: 0};
            drawMap();
        }

        function toggleRobotTrail() {
            showTrail = !showTrail;
            showNotification(`Trajectoire: ${showTrail ? 'ON' : 'OFF'}`, 'info');
        }

        // Draw map
        function drawMap() {
            const ctx = mapCtx;
            const w = mapCanvas.width;
            const h = mapCanvas.height;
            
            ctx.fillStyle = '#0f0f1e';
            ctx.fillRect(0, 0, w, h);
            
            ctx.save();
            ctx.translate(w/2, h/2);
            ctx.scale(mapZoom, mapZoom);
            ctx.translate(mapOffset.x / mapZoom, mapOffset.y / mapZoom);
            
            // Draw grid
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
            ctx.lineWidth = 0.5 / mapZoom;
            
            const gridSize = 2.0;
            const gridExtent = 30;
            
            for (let x = -gridExtent; x <= gridExtent; x += gridSize) {
                ctx.beginPath();
                ctx.moveTo(x, -gridExtent);
                ctx.lineTo(x, gridExtent);
                ctx.stroke();
            }
            
            for (let y = -gridExtent; y <= gridExtent; y += gridSize) {
                ctx.beginPath();
                ctx.moveTo(-gridExtent, y);
                ctx.lineTo(gridExtent, y);
                ctx.stroke();
            }
            
            // Draw axis
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
            ctx.lineWidth = 1 / mapZoom;
            ctx.beginPath();
            ctx.moveTo(-gridExtent, 0);
            ctx.lineTo(gridExtent, 0);
            ctx.moveTo(0, -gridExtent);
            ctx.lineTo(0, gridExtent);
            ctx.stroke();
            
            // Draw robot trail
            if (showTrail && robotTrail.length > 1) {
                ctx.strokeStyle = 'rgba(102, 126, 234, 0.6)';
                ctx.lineWidth = 1.5 / mapZoom;
                ctx.beginPath();
                ctx.moveTo(robotTrail[0].x, robotTrail[0].y);
                for (let i = 1; i < robotTrail.length; i++) {
                    ctx.lineTo(robotTrail[i].x, robotTrail[i].y);
                }
                ctx.stroke();
            }
            
            // Draw waypoints
            waypoints.forEach(wp => {
                const x = parseFloat(wp.x);
                const y = parseFloat(wp.y);
                
                ctx.fillStyle = '#ef4444';
                ctx.beginPath();
                ctx.arc(x, -y, 0.3, 0, Math.PI * 2);
                ctx.fill();
                
                ctx.fillStyle = 'white';
                ctx.font = `${12 / mapZoom}px Arial`;
                ctx.textAlign = 'center';
                ctx.fillText(wp.name, x, -y - 0.5);
            });
            
            drawHospitalLayout(ctx);
            
            // Draw robot
            if (robotPose) {
                const rx = robotPose.x;
                const ry = -robotPose.y;
                
                ctx.save();
                ctx.translate(rx, ry);
                ctx.rotate(-robotPose.theta);
                
                ctx.fillStyle = '#4ade80';
                ctx.beginPath();
                ctx.arc(0, 0, 0.2, 0, Math.PI * 2);
                ctx.fill();
                
                ctx.strokeStyle = '#22c55e';
                ctx.lineWidth = 1 / mapZoom;
                ctx.beginPath();
                ctx.moveTo(0, 0);
                ctx.lineTo(0.3, 0);
                ctx.stroke();
                
                ctx.restore();
            }
            
            ctx.restore();
        }

        function drawHospitalLayout(ctx) {
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
            ctx.lineWidth = 0.8 / mapZoom;
            
            ctx.strokeRect(-15, -15, 30, 30);
            ctx.strokeRect(-10, 6, 6, 6);
            ctx.strokeRect(-4, 6, 6, 6);
            ctx.strokeRect(4, 6, 6, 6);
            ctx.strokeRect(-10, -12, 6, 6);
            ctx.strokeRect(-4, -12, 6, 6);
            ctx.strokeRect(4, -12, 6, 6);
            
            ctx.strokeStyle = 'rgba(102, 126, 234, 0.8)';
            ctx.strokeRect(-13, -3, 6, 6);
        }

        // Draw laser
        function drawLaser(scanData) {
            const ctx = laserCtx;
            const w = laserCanvas.width;
            const h = laserCanvas.height;
            const cx = w / 2;
            const cy = h / 2;
            const radius = Math.min(w, h) / 2;
            
            ctx.fillStyle = '#0f0f1e';
            ctx.fillRect(0, 0, w, h);
            
            if (!scanData || !scanData.ranges) return;
            
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
            ctx.lineWidth = 1;
            
            for (let i = 1; i <= 4; i++) {
                ctx.beginPath();
                ctx.arc(cx, cy, (radius * i) / 4, 0, Math.PI * 2);
                ctx.stroke();
            }
            
            ctx.fillStyle = '#ef4444';
            const ranges = scanData.ranges;
            const angleMin = scanData.angle_min;
            const angleIncrement = scanData.angle_increment;
            
            for (let i = 0; i < ranges.length; i++) {
                const range = ranges[i];
                if (range > 0.1 && range < 12) {
                    const angle = angleMin + i * angleIncrement;
                    const x = cx + Math.cos(angle) * (range / 12) * radius;
                    const y = cy + Math.sin(angle) * (range / 12) * radius;
                    
                    ctx.beginPath();
                    ctx.arc(x, y, 2, 0, Math.PI * 2);
                    ctx.fill();
                }
            }
            
            ctx.fillStyle = '#4ade80';
            ctx.beginPath();
            ctx.arc(cx, cy, 4, 0, Math.PI * 2);
            ctx.fill();
        }

        // Update data from server
        async function updateData() {
            try {
                const response = await fetch('/get_slam_data');
                const data = await response.json();
                
                if (data.pose) {
                    robotPose = data.pose;
                    document.getElementById('pos-x').textContent = robotPose.x.toFixed(2);
                    document.getElementById('pos-y').textContent = robotPose.y.toFixed(2);
                    document.getElementById('rotation').textContent = 
                        (robotPose.theta * 180 / Math.PI).toFixed(1) + '°';
                    
                    robotTrail.push({x: robotPose.x, y: -robotPose.y});
                    if (robotTrail.length > 100) robotTrail.shift();
                }
                
                if (data.velocity !== undefined) {
                    document.getElementById('velocity').textContent = data.velocity.toFixed(2);
                }
                
                if (data.laser) {
                    drawLaser(data.laser);
                }
                
                if (data.stats) {
                    document.getElementById('distance-traveled').textContent = 
                        data.stats.distance.toFixed(1);
                    document.getElementById('explored-area').textContent = 
                        data.stats.area.toFixed(0);
                    document.getElementById('scan-quality').textContent = 
                        data.stats.quality + '%';
                    document.getElementById('map-status').textContent = 
                        `🗺️ Cartographie: ${data.stats.coverage}%`;
                }
                
                drawMap();
                lastUpdateTime = Date.now();
            } catch (error) {
                console.error('Error updating data:', error);
            }
        }

        // Initial setup
        document.addEventListener('DOMContentLoaded', () => {
            setInterval(updateData, 100);
            drawMap();
            showNotification('Interface chargée - Système prêt', 'success');
        });
    </script>
</body>
</html>
"""

def get_local_ip():
    """Get the local IP address of the machine"""
    try:
        # Create a socket to determine the local IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception:
        return "localhost"

class SLAMWebController(Node):
    def __init__(self):
        super().__init__('slam_web_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # State
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.current_command = 'stop'
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.velocity = 0.0
        self.map_data = None
        self.laser_data = None
        self.distance_traveled = 0.0
        self.last_pose = None
        self.loop_closures = 0
        
        # Hospital locations
        self.hospital_locations = {
            'reception': {'x': -12.0, 'y': 0.0, 'name': 'Réception'},
            'emergency': {'x': -11.0, 'y': -8.0, 'name': 'Urgences'},
            'pharmacy': {'x': 10.0, 'y': -8.0, 'name': 'Pharmacie'},
            'lab': {'x': -1.0, 'y': -9.0, 'name': 'Laboratoire'},
            'consult1': {'x': -11.0, 'y': 7.0, 'name': 'Consultation 1'},
            'consult2': {'x': -1.0, 'y': 7.0, 'name': 'Consultation 2'},
            'consult3': {'x': 10.0, 'y': 7.0, 'name': 'Consultation 3'},
            'entrance': {'x': 0.0, 'y': 14.0, 'name': 'Entrée Principale'}
        }
        
        # Timer
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        self.get_logger().info('🎮 SLAM Web Controller initialized')

    def odom_callback(self, msg):
        self.robot_pose['x'] = msg.pose.pose.position.x
        self.robot_pose['y'] = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_pose['theta'] = math.atan2(siny_cosp, cosy_cosp)
        
        self.velocity = math.sqrt(
            msg.twist.twist.linear.x**2 + 
            msg.twist.twist.linear.y**2
        )
        
        if self.last_pose:
            dx = self.robot_pose['x'] - self.last_pose['x']
            dy = self.robot_pose['y'] - self.last_pose['y']
            self.distance_traveled += math.sqrt(dx*dx + dy*dy)
        self.last_pose = self.robot_pose.copy()

    def map_callback(self, msg):
        self.map_data = {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'data': list(msg.data)
        }

    def scan_callback(self, msg):
        self.laser_data = {
            'ranges': list(msg.ranges),
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment
        }

    def set_command(self, command):
        self.current_command = command

    def publish_velocity(self):
        msg = Twist()
        
        commands = {
            'forward': (self.linear_speed, 0.0),
            'backward': (-self.linear_speed, 0.0),
            'left': (0.0, self.angular_speed),
            'right': (0.0, -self.angular_speed),
            'forward_left': (self.linear_speed, self.angular_speed),
            'forward_right': (self.linear_speed, -self.angular_speed),
            'backward_left': (-self.linear_speed, self.angular_speed),
            'backward_right': (-self.linear_speed, -self.angular_speed),
            'stop': (0.0, 0.0)
        }
        
        if self.current_command in commands:
            msg.linear.x, msg.angular.z = commands[self.current_command]
            self.cmd_vel_pub.publish(msg)

    def get_slam_stats(self):
        explored_area = 0
        if self.map_data:
            explored_area = sum(1 for d in self.map_data['data'] if d > 0) * \
                          (self.map_data['resolution'] ** 2)
        
        scan_quality = 0
        if self.laser_data:
            valid_scans = sum(1 for r in self.laser_data['ranges'] if 0.3 < r < 12)
            scan_quality = int(valid_scans / len(self.laser_data['ranges']) * 100)
        
        return {
            'distance': self.distance_traveled,
            'area': explored_area,
            'quality': scan_quality,
            'coverage': min(100, int(explored_area / 100)),
            'loop_closures': self.loop_closures
        }

    def navigate_to_location(self, location_id):
        if location_id in self.hospital_locations:
            loc = self.hospital_locations[location_id]
            
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = loc['x']
            goal.pose.position.y = loc['y']
            goal.pose.orientation.w = 1.0
            
            self.goal_pub.publish(goal)
            self.get_logger().info(f'🚀 Navigation vers {loc["name"]} ({loc["x"]}, {loc["y"]})')
            return True
        
        return False

# Flask app
app = Flask(__name__)
CORS(app)
slam_controller = None
waypoints_db = []

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/control', methods=['POST'])
def control():
    data = request.json
    command = data.get('command', '')
    valid_commands = ['forward', 'backward', 'left', 'right', 
                     'forward_left', 'forward_right', 
                     'backward_left', 'backward_right', 'stop']
    
    if command in valid_commands:
        slam_controller.set_command(command)
        return jsonify({'status': 'success', 'command': command})
    
    return jsonify({'status': 'error', 'message': 'Unknown command'})

@app.route('/set_speed', methods=['POST'])
def set_speed():
    data = request.json
    if 'linear' in data:
        slam_controller.linear_speed = float(data['linear'])
    if 'angular' in data:
        slam_controller.angular_speed = float(data['angular'])
    return jsonify({'status': 'success'})

@app.route('/get_slam_data', methods=['GET'])
def get_slam_data():
    return jsonify({
        'pose': slam_controller.robot_pose,
        'velocity': slam_controller.velocity,
        'map': slam_controller.map_data,
        'laser': slam_controller.laser_data,
        'stats': slam_controller.get_slam_stats()
    })

@app.route('/add_waypoint', methods=['POST'])
def add_waypoint():
    waypoint = request.json
    waypoints_db.append(waypoint)
    return jsonify({'status': 'success'})

@app.route('/goto_waypoint', methods=['POST'])
def goto_waypoint():
    waypoint = request.json
    
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = slam_controller.get_clock().now().to_msg()
    goal.pose.position.x = float(waypoint['x'])
    goal.pose.position.y = float(waypoint['y'])
    goal.pose.orientation.w = 1.0
    
    slam_controller.goal_pub.publish(goal)
    return jsonify({'status': 'success'})

@app.route('/goto_location', methods=['POST'])
def goto_location():
    data = request.json
    location_id = data.get('location', '')
    
    if slam_controller.navigate_to_location(location_id):
        return jsonify({'status': 'success', 'location': location_id})
    
    return jsonify({'status': 'error', 'message': 'Location not found'})

@app.route('/save_map', methods=['POST'])
def save_map():
    try:
        maps_dir = os.path.expanduser('~/Last_ros/src/my_robot_controller/maps')
        os.makedirs(maps_dir, exist_ok=True)
        
        result = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', f'{maps_dir}/hospital_map'],
            capture_output=True,
            text=True,
            timeout=10
        )
        
        if result.returncode == 0:
            return jsonify({'status': 'success', 'message': f'Carte sauvegardée dans {maps_dir}'})
        else:
            return jsonify({'status': 'error', 'message': result.stderr})
            
    except subprocess.TimeoutExpired:
        return jsonify({'status': 'error', 'message': 'Timeout lors de la sauvegarde'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/reset_pose', methods=['POST'])
def reset_pose():
    slam_controller.distance_traveled = 0.0
    slam_controller.last_pose = None
    return jsonify({'status': 'success'})

@app.route('/clear_map', methods=['POST'])
def clear_map():
    waypoints_db.clear()
    return jsonify({'status': 'success'})

@app.route('/relocalize', methods=['POST'])
def relocalize():
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = slam_controller.get_clock().now().to_msg()
    initial_pose.pose.pose.position.x = slam_controller.robot_pose['x']
    initial_pose.pose.pose.position.y = slam_controller.robot_pose['y']
    initial_pose.pose.pose.orientation.w = 1.0
    
    slam_controller.initial_pose_pub.publish(initial_pose)
    slam_controller.loop_closures += 1
    
    return jsonify({'status': 'success'})

def run_flask():
    """Run Flask with better network configuration"""
    # Get local IP
    local_ip = get_local_ip()
    
    print("\n" + "="*80)
    print("🌐 DÉMARRAGE DU SERVEUR WEB")
    print("="*80)
    print(f"🔗 Accès depuis la VM:        http://localhost:5000")
    print(f"🔗 Accès depuis l'hôte:       http://{local_ip}:5000")
    print(f"🔗 Accès depuis le réseau:    http://{local_ip}:5000")
    print("="*80)
    print("💡 Assurez-vous que:")
    print("   1. Votre VM est en mode Bridge ou NAT avec port forwarding")
    print("   2. Le pare-feu autorise le port 5000")
    print("   3. Utilisez l'IP ci-dessus depuis votre machine hôte")
    print("="*80 + "\n")
    
    # Run Flask with proper configuration for external access
    app.run(
        host='0.0.0.0',  # Listen on all interfaces
        port=5000,
        debug=False,
        use_reloader=False,
        threaded=True
    )

def main(args=None):
    global slam_controller
    
    rclpy.init(args=args)
    slam_controller = SLAMWebController()

    # Get network info
    local_ip = get_local_ip()

    # Start Flask in a separate thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    print("\n" + "="*80)
    print("🏥 SYSTÈME DE NAVIGATION HOSPITALIÈRE SLAM")
    print("="*80)
    print(f"📡 Interface Web (VM):         http://localhost:5000")
    print(f"📡 Interface Web (Hôte):       http://{local_ip}:5000")
    print(f"📡 Interface Web (Réseau):     http://{local_ip}:5000")
    print("="*80)
    print("✨ Fonctionnalités:")
    print("   • Contrôle manuel du robot (clavier/boutons)")
    print("   • Cartographie SLAM temps réel")
    print("   • Visualisation LiDAR 360°")
    print("   • Navigation vers 8 salles prédéfinies")
    print("   • Waypoints personnalisables")
    print("   • Sauvegarde de carte")
    print("   • Indicateur de connexion")
    print("="*80)
    print("🎮 Contrôles:")
    print("   • Clavier: W/A/S/D ou flèches directionnelles")
    print("   • Souris: Clic sur carte pour waypoints")
    print("   • Boutons: Interface graphique")
    print("="*80)
    print("🔧 Configuration réseau:")
    print("   • Mode recommandé: Bridge")
    print("   • Port: 5000 (TCP)")
    print("   • Pare-feu: Autoriser port 5000")
    print("="*80 + "\n")

    try:
        rclpy.spin(slam_controller)
    except KeyboardInterrupt:
        print("\n🛑 Arrêt du système...")
    finally:
        slam_controller.destroy_node()
        rclpy.shutdown()
        print("✅ Système arrêté proprement")

if __name__ == '__main__':
    main()