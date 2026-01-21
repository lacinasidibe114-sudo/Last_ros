#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Empty
from flask import Flask, render_template_string, request, jsonify
from flask_cors import CORS
import threading
import numpy as np
import math
import subprocess
import os
import time
import asyncio

# Template HTML avec Nav2 intégré
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🏥 Système de Navigation Hospitalière Nav2</title>
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
            overflow-y: auto;
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

        .status-item.warning {
            background: #fbbf24;
        }

        .status-item.error {
            background: #ef4444;
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

        .waypoint-item.navigating {
            border-left-color: #fbbf24;
            background: #fef3c7;
            animation: pulse 1s ease-in-out infinite;
        }

        .waypoint-info {
            flex: 1;
        }

        .waypoint-coords {
            font-size: 0.8em;
            color: #666;
            margin-top: 3px;
        }

        .waypoint-status {
            font-size: 0.75em;
            margin-top: 5px;
            padding: 3px 8px;
            border-radius: 10px;
            display: inline-block;
        }

        .waypoint-status.pending {
            background: #e5e7eb;
            color: #6b7280;
        }

        .waypoint-status.active {
            background: #fef3c7;
            color: #92400e;
        }

        .waypoint-status.completed {
            background: #d1fae5;
            color: #065f46;
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

        .action-btn.cancel {
            background: #fbbf24;
            color: white;
        }

        .action-btn:hover {
            transform: scale(1.1);
        }

        .action-btn:disabled {
            opacity: 0.5;
            cursor: not-allowed;
        }

        .nav-actions {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 8px;
            margin-top: 15px;
        }

        .nav-btn {
            padding: 12px;
            border: none;
            border-radius: 10px;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.3s ease;
            font-size: 0.9em;
        }

        .nav-btn.save {
            background: linear-gradient(135deg, #4ade80 0%, #22c55e 100%);
            color: white;
        }

        .nav-btn.reset {
            background: linear-gradient(135deg, #f59e0b 0%, #d97706 100%);
            color: white;
        }

        .nav-btn.clear {
            background: linear-gradient(135deg, #ef4444 0%, #dc2626 100%);
            color: white;
        }

        .nav-btn.localize {
            background: linear-gradient(135deg, #8b5cf6 0%, #7c3aed 100%);
            color: white;
        }

        .nav-btn.cancel-nav {
            background: linear-gradient(135deg, #fb923c 0%, #f97316 100%);
            color: white;
            grid-column: 1 / -1;
        }

        .nav-btn:hover:not(:disabled) {
            transform: translateY(-3px);
            box-shadow: 0 5px 15px rgba(0, 0, 0, 0.3);
        }

        .nav-btn:disabled {
            opacity: 0.5;
            cursor: not-allowed;
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

        .location-btn:hover:not(:disabled) {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(59, 130, 246, 0.4);
        }

        .location-btn:disabled {
            opacity: 0.5;
            cursor: not-allowed;
        }

        .location-btn em {
            font-size: 1.2em;
        }

        .location-name {
            font-size: 0.8em;
        }

        .progress-bar {
            width: 100%;
            height: 6px;
            background: #e5e7eb;
            border-radius: 3px;
            overflow: hidden;
            margin-top: 8px;
        }

        .progress-fill {
            height: 100%;
            background: linear-gradient(90deg, #4ade80, #22c55e);
            transition: width 0.3s ease;
            border-radius: 3px;
        }

        .nav-status-panel {
            background: #f8f9fa;
            padding: 15px;
            border-radius: 10px;
            margin-top: 15px;
        }

        .nav-status-title {
            font-weight: bold;
            color: #667eea;
            margin-bottom: 10px;
            display: flex;
            align-items: center;
            gap: 8px;
        }

        .nav-status-info {
            font-size: 0.9em;
            color: #666;
            line-height: 1.6;
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
    </style>
</head>
<body>
    <div class="dashboard">
        <!-- Header -->
        <div class="panel header">
            <h1>
                <span>🏥</span>
                Navigation Hospitalière Nav2
                <span>🤖</span>
            </h1>
            <div class="status-bar">
                <div class="status-item" id="nav2-status">● Nav2: Initialisation...</div>
                <div class="status-item active" id="robot-status">⚡ Robot Prêt</div>
                <div class="status-item" id="map-status">🗺️ Carte Chargée</div>
                <div class="status-item" id="localization-status">📍 Localisation: OK</div>
            </div>
        </div>

        <!-- Panneau de Contrôle -->
        <div class="panel">
            <div class="section-title">
                <span>🎯</span>
                Mode de Navigation
            </div>

            <div class="mode-selector">
                <button class="mode-btn active" id="mode-manual" onclick="setMode('manual')">
                    🎮 Manuel
                </button>
                <button class="mode-btn" id="mode-auto" onclick="setMode('auto')">
                    🤖 Auto Nav2
                </button>
            </div>

            <div id="manual-controls">
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

            <div class="nav-status-panel" id="nav-status-panel" style="display:none;">
                <div class="nav-status-title">
                    <span>🚀</span>
                    <span id="nav-status-title-text">Navigation Active</span>
                </div>
                <div class="nav-status-info" id="nav-status-info">
                    État: En cours...<br>
                    Distance restante: <span id="remaining-distance">0.0</span> m<br>
                    Temps estimé: <span id="estimated-time">--</span> s
                </div>
                <div class="progress-bar">
                    <div class="progress-fill" id="nav-progress" style="width: 0%"></div>
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

        <!-- Carte Nav2 -->
        <div class="panel">
            <div class="section-title">
                <span>🗺️</span>
                Carte Nav2 + Planification
            </div>

            <div id="map-container">
                <canvas id="map-canvas"></canvas>
                <div class="map-overlay">
                    <div style="color: #4ade80; font-weight: bold;">🤖 Robot</div>
                    <div style="color: #ef4444;">📍 Waypoints</div>
                    <div style="color: #fbbf24;">🎯 Objectif</div>
                    <div style="color: #8b5cf6;">━━ Chemin Planifié</div>
                    <div style="margin-top: 8px; padding-top: 8px; border-top: 1px solid #333; font-size: 0.8em;">
                        <div>Distance: <span id="distance-traveled">0.0</span> m</div>
                        <div>Costmap: <span id="costmap-status">OK</span></div>
                    </div>
                </div>
                <div class="map-controls">
                    <button class="map-btn" onclick="zoomIn()" title="Zoom In">+</button>
                    <button class="map-btn" onclick="zoomOut()" title="Zoom Out">−</button>
                    <button class="map-btn" onclick="centerMap()" title="Centrer">⌖</button>
                    <button class="map-btn" onclick="togglePath()" title="Trajectoire">↗</button>
                </div>
            </div>

            <div class="nav-actions">
                <button class="nav-btn save" onclick="saveMap()">
                    💾 Sauvegarder
                </button>
                <button class="nav-btn localize" onclick="setInitialPose()">
                    📍 Pose Initiale
                </button>
                <button class="nav-btn reset" onclick="clearCostmaps()">
                    🧹 Clear Costmaps
                </button>
                <button class="nav-btn clear" onclick="clearWaypoints()">
                    🗑️ Effacer Points
                </button>
                <button class="nav-btn cancel-nav" id="cancel-nav-btn" onclick="cancelNavigation()" disabled>
                    ⛔ Annuler Navigation
                </button>
            </div>

            <div class="stats-row">
                <div class="stat-item">
                    <div class="stat-value" id="waypoints-count">0</div>
                    <div class="stat-label">Waypoints</div>
                </div>
                <div class="stat-item">
                    <div class="stat-value" id="nav-goals-completed">0</div>
                    <div class="stat-label">Objectifs</div>
                </div>
                <div class="stat-item">
                    <div class="stat-value" id="nav-status-display">Idle</div>
                    <div class="stat-label">État Nav2</div>
                </div>
            </div>
        </div>

        <!-- Waypoints et Navigation -->
        <div class="panel">
            <div class="section-title">
                <span>📍</span>
                Points de Navigation Nav2
            </div>

            <div style="margin-bottom: 12px; padding: 12px; background: #e0e7ff; border-radius: 8px; color: #667eea; font-weight: 600; font-size: 0.9em;">
                💡 Mode Auto: Cliquez sur la carte pour définir un objectif
            </div>

            <div class="waypoint-list" id="waypoint-list">
                <div style="text-align: center; color: #999; padding: 30px; font-size: 0.9em;">
                    Aucun waypoint défini<br>
                    <small>Cliquez sur la carte en mode Auto pour naviguer</small>
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
                <span>📋</span>
                Actions Rapides
            </div>

            <div class="nav-actions">
                <button class="nav-btn" style="background: linear-gradient(135deg, #06b6d4 0%, #0891b2 100%); color: white;" onclick="followWaypoints()">
                    🚶 Suivre Tous
                </button>
                <button class="nav-btn" style="background: linear-gradient(135deg, #ec4899 0%, #db2777 100%); color: white;" onclick="returnHome()">
                    🏠 Retour Base
                </button>
            </div>
        </div>
    </div>

    <!-- Notification -->
    <div class="notification" id="notification"></div>

    <script>
        let currentMode = 'manual';
        let currentCommand = 'stop';
        let waypoints = [];
        let currentGoalIndex = -1;
        let robotPose = {x: 0, y: 0, theta: 0};
        let navigationActive = false;
        let plannedPath = [];
        let mapZoom = 20.0;
        let mapOffset = {x: 0, y: 0};
        let showPath = true;
        let robotTrail = [];
        let goalsCompleted = 0;

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

        // Mode switching
        function setMode(mode) {
            currentMode = mode;
            document.getElementById('mode-manual').classList.toggle('active', mode === 'manual');
            document.getElementById('mode-auto').classList.toggle('active', mode === 'auto');
            
            if (mode === 'manual') {
                document.getElementById('manual-controls').style.display = 'block';
                showNotification('Mode Manuel activé - Contrôle direct', 'info');
            } else {
                document.getElementById('manual-controls').style.display = 'none';
                sendCommand('stop');
                showNotification('Mode Auto Nav2 activé - Navigation autonome', 'info');
            }
        }

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
            if (currentMode !== 'manual') return;
            
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
                }
            } catch (error) {
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
                if (currentMode === 'manual') {
                    sendCommand(button.dataset.command);
                }
            });
            
            button.addEventListener('mouseup', () => {
                if (currentMode === 'manual') {
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
            if (keyMap[e.key] && currentMode === 'manual') {
                e.preventDefault();
                if (!activeKeys.has(e.key)) {
                    activeKeys.add(e.key);
                    sendCommand(keyMap[e.key]);
                }
            }
        });

        document.addEventListener('keyup', (e) => {
            if (keyMap[e.key] && currentMode === 'manual') {
                e.preventDefault();
                activeKeys.delete(e.key);
                if (activeKeys.size === 0) {
                    sendCommand('stop');
                }
            }
        });

        // Map interaction
        mapCanvas.addEventListener('click', async (e) => {
            if (currentMode !== 'auto') return;
            
            const rect = mapCanvas.getBoundingClientRect();
            const x = (e.clientX - rect.left - mapCanvas.width/2 + mapOffset.x) / mapZoom;
            const y = (e.clientY - rect.top - mapCanvas.height/2 + mapOffset.y) / mapZoom;
            
            await navigateToGoal(x, -y);
        });

        async function navigateToGoal(x, y) {
            try {
                const response = await fetch('/navigate_to_pose', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({x: x.toFixed(2), y: y.toFixed(2)})
                });
                
                const data = await response.json();
                if (data.status === 'success') {
                    showNotification(`Navigation vers (${x.toFixed(2)}, ${y.toFixed(2)})`, 'info');
                    navigationActive = true;
                    document.getElementById('cancel-nav-btn').disabled = false;
                } else {
                    showNotification('Erreur de navigation: ' + data.message, 'error');
                }
            } catch (error) {
                showNotification('Erreur de connexion', 'error');
            }
        }

        async function goToLocation(location) {
            if (currentMode !== 'auto') {
                showNotification('Passez en mode Auto pour naviguer', 'warning');
                return;
            }
            
            try {
                const response = await fetch('/goto_location', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({location: location})
                });
                
                const data = await response.json();
                if (data.status === 'success') {
                    showNotification(`Navigation vers ${location}`, 'success');
                    navigationActive = true;
                    document.getElementById('cancel-nav-btn').disabled = false;
                }
            } catch (error) {
                showNotification('Erreur navigation', 'error');
            }
        }

        async function cancelNavigation() {
            try {
                const response = await fetch('/cancel_navigation', {method: 'POST'});
                const data = await response.json();
                if (data.status === 'success') {
                    showNotification('Navigation annulée', 'info');
                    navigationActive = false;
                    document.getElementById('cancel-nav-btn').disabled = true;
                    document.getElementById('nav-status-panel').style.display = 'none';
                }
            } catch (error) {
                showNotification('Erreur annulation', 'error');
            }
        }

        async function setInitialPose() {
            showNotification('Utilisez RViz pour définir la pose initiale', 'info');
        }

        async function clearCostmaps() {
            try {
                const response = await fetch('/clear_costmaps', {method: 'POST'});
                const data = await response.json();
                if (data.status === 'success') {
                    showNotification('Costmaps effacés', 'success');
                }
            } catch (error) {
                showNotification('Erreur effacement costmaps', 'error');
            }
        }

        async function saveMap() {
            showNotification('Sauvegarde de la carte...', 'info');
            try {
                const response = await fetch('/save_map', {method: 'POST'});
                const data = await response.json();
                if (data.status === 'success') {
                    showNotification('Carte sauvegardée!', 'success');
                }
            } catch (error) {
                showNotification('Erreur sauvegarde', 'error');
            }
        }

        function clearWaypoints() {
            waypoints = [];
            updateWaypointList();
            showNotification('Waypoints effacés', 'info');
        }

        function followWaypoints() {
            if (waypoints.length === 0) {
                showNotification('Aucun waypoint défini', 'warning');
                return;
            }
            showNotification(`Suivi de ${waypoints.length} waypoints`, 'info');
        }

        function returnHome() {
            navigateToGoal(0, 0);
            showNotification('Retour à la base (0, 0)', 'info');
        }

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

        function togglePath() {
            showPath = !showPath;
            showNotification(`Chemin: ${showPath ? 'ON' : 'OFF'}`, 'info');
            drawMap();
        }

        // Draw map with hospital layout
        function drawMap() {
            const ctx = mapCtx;
            const w = mapCanvas.width;
            const h = mapCanvas.height;
            
            // Clear canvas
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
            
            // Draw hospital layout based on your world file
            drawHospitalLayout(ctx);
            
            // Draw planned path
            if (showPath && plannedPath.length > 1) {
                ctx.strokeStyle = '#8b5cf6';
                ctx.lineWidth = 2 / mapZoom;
                ctx.setLineDash([0.2, 0.1]);
                ctx.beginPath();
                ctx.moveTo(plannedPath[0].x, -plannedPath[0].y);
                for (let i = 1; i < plannedPath.length; i++) {
                    ctx.lineTo(plannedPath[i].x, -plannedPath[i].y);
                }
                ctx.stroke();
                ctx.setLineDash([]);
            }
            
            // Draw robot trail
            if (robotTrail.length > 1) {
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
                
                ctx.fillStyle = wp.active ? '#fbbf24' : '#ef4444';
                ctx.beginPath();
                ctx.arc(x, -y, 0.3, 0, Math.PI * 2);
                ctx.fill();
            });
            
            // Draw robot
            if (robotPose) {
                const rx = robotPose.x;
                const ry = -robotPose.y;
                
                ctx.save();
                ctx.translate(rx, ry);
                ctx.rotate(-robotPose.theta);
                
                // Robot body
                ctx.fillStyle = '#4ade80';
                ctx.beginPath();
                ctx.arc(0, 0, 0.2, 0, Math.PI * 2);
                ctx.fill();
                
                // Robot direction
                ctx.strokeStyle = '#22c55e';
                ctx.lineWidth = 2 / mapZoom;
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
            
            // Outer walls (30x30m)
            ctx.strokeRect(-15, -15, 30, 30);
            
            // Rooms (from your world file coordinates)
            // Reception area
            ctx.strokeRect(-13, -3, 6, 6);
            
            // Consultation 1 (Nord-Ouest)
            ctx.strokeRect(-10, 6, 6, 6);
            
            // Consultation 2 (Centre-Nord)
            ctx.strokeRect(-4, 6, 6, 6);
            
            // Consultation 3 (Nord-Est)
            ctx.strokeRect(4, 6, 6, 6);
            
            // Emergency (Sud-Ouest)
            ctx.strokeRect(-10, -12, 6, 6);
            
            // Laboratory (Sud-Centre)
            ctx.strokeRect(-4, -12, 6, 6);
            
            // Pharmacy (Sud-Est)
            ctx.strokeRect(4, -12, 6, 6);
            
            // Label the rooms
            ctx.fillStyle = 'rgba(255, 255, 255, 0.7)';
            ctx.font = `${10 / mapZoom}px Arial`;
            ctx.textAlign = 'center';
            
            ctx.fillText('Réception', -10, -3);
            ctx.fillText('Consult 1', -7, 9);
            ctx.fillText('Consult 2', -1, 9);
            ctx.fillText('Consult 3', 7, 9);
            ctx.fillText('Urgences', -7, -9);
            ctx.fillText('Lab', -1, -9);
            ctx.fillText('Pharmacie', 7, -9);
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
            
            // Draw distance circles
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
            ctx.lineWidth = 1;
            
            for (let i = 1; i <= 4; i++) {
                ctx.beginPath();
                ctx.arc(cx, cy, (radius * i) / 4, 0, Math.PI * 2);
                ctx.stroke();
            }
            
            // Draw laser points
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
            
            // Draw robot center
            ctx.fillStyle = '#4ade80';
            ctx.beginPath();
            ctx.arc(cx, cy, 4, 0, Math.PI * 2);
            ctx.fill();
        }

        // Update data from server
        async function updateData() {
            try {
                const response = await fetch('/get_nav_data');
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
                
                if (data.nav_status) {
                    updateNavStatus(data.nav_status);
                }
                
                if (data.planned_path) {
                    plannedPath = data.planned_path;
                }
                
                if (data.stats) {
                    document.getElementById('distance-traveled').textContent = 
                        data.stats.distance.toFixed(1);
                    document.getElementById('nav-goals-completed').textContent = 
                        data.stats.goals_completed;
                }
                
                drawMap();
            } catch (error) {
                console.error('Error updating data:', error);
            }
        }

        function updateNavStatus(status) {
            const statusDisplay = document.getElementById('nav-status-display');
            const nav2Status = document.getElementById('nav2-status');
            const locStatus = document.getElementById('localization-status');
            
            statusDisplay.textContent = status.state || 'Idle';
            
            if (status.is_navigating) {
                document.getElementById('nav-status-panel').style.display = 'block';
                document.getElementById('nav-status-title-text').textContent = 
                    `Navigation Active - ${status.state}`;
                document.getElementById('remaining-distance').textContent = 
                    (status.remaining_distance || 0).toFixed(2);
                document.getElementById('estimated-time').textContent = 
                    (status.estimated_time || 0).toFixed(0);
                
                const progress = Math.min(100, Math.max(0, 
                    (1 - (status.remaining_distance / (status.total_distance || 1))) * 100
                ));
                document.getElementById('nav-progress').style.width = progress + '%';
                
                nav2Status.className = 'status-item active';
                nav2Status.textContent = '● Nav2: Navigating';
            } else {
                document.getElementById('nav-status-panel').style.display = 'none';
                nav2Status.className = 'status-item';
                nav2Status.textContent = '● Nav2: Ready';
                document.getElementById('cancel-nav-btn').disabled = true;
            }
            
            if (status.localization_quality && status.localization_quality < 50) {
                locStatus.className = 'status-item warning';
                locStatus.textContent = '📍 Localisation: Faible';
            } else {
                locStatus.className = 'status-item';
                locStatus.textContent = '📍 Localisation: OK';
            }
        }

        // Initial setup
        document.addEventListener('DOMContentLoaded', () => {
            setInterval(updateData, 100);
            drawMap();
        });
    </script>
</body>
</html>
'''

class Nav2WebController(Node):
    def __init__(self):
        super().__init__('nav2_web_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.global_plan_sub = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        
        # Action Clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        # Services
        self.clear_global_costmap_client = self.create_client(Empty, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local_costmap_client = self.create_client(Empty, '/local_costmap/clear_entirely_local_costmap')
        
        # State
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.current_command = 'stop'
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.velocity = 0.0
        self.map_data = None
        self.laser_data = None
        self.planned_path = []
        self.distance_traveled = 0.0
        self.last_pose = None
        self.goals_completed = 0
        self.current_goal_handle = None
        self.is_navigating = False
        self.nav_start_time = None
        self.nav_start_pose = None
        
        # Hospital locations - adaptées à votre monde
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
        
        self.get_logger().info('🎮 Nav2 Web Controller initialized')
        self.get_logger().info('🏥 Hospital locations loaded')
        for loc_id, loc in self.hospital_locations.items():
            self.get_logger().info(f'  • {loc["name"]}: ({loc["x"]}, {loc["y"]})')

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

    def plan_callback(self, msg):
        self.planned_path = [
            {'x': pose.pose.position.x, 'y': pose.pose.position.y}
            for pose in msg.poses
        ]

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

    def navigate_to_pose(self, x, y, theta=0.0):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = math.cos(float(theta) / 2)
        goal_msg.pose.pose.orientation.z = math.sin(float(theta) / 2)
        
        self.get_logger().info(f'🚀 Sending goal: ({x:.2f}, {y:.2f})')
        
        self.nav_start_time = time.time()
        self.nav_start_pose = self.robot_pose.copy()
        self.is_navigating = True
        
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)
        
        return True

    def nav_goal_response_callback(self, future):
        self.current_goal_handle = future.result()
        if not self.current_goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.is_navigating = False
            return
        
        self.get_logger().info('Goal accepted')
        result_future = self.current_goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Could process feedback here
        pass

    def nav_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ Goal reached successfully!')
            self.goals_completed += 1
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('⚠️ Goal canceled')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('❌ Goal aborted')
        
        self.is_navigating = False
        self.current_goal_handle = None

    def cancel_navigation(self):
        if self.current_goal_handle:
            self.get_logger().info('Canceling current goal...')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.get_logger().info('Goal canceled'))
            self.is_navigating = False
            return True
        return False

    def clear_costmaps(self):
        if self.clear_global_costmap_client.service_is_ready():
            self.clear_global_costmap_client.call_async(Empty.Request())
        if self.clear_local_costmap_client.service_is_ready():
            self.clear_local_costmap_client.call_async(Empty.Request())
        self.get_logger().info('Costmaps cleared')

    def get_nav_status(self):
        remaining_dist = 0.0
        total_dist = 0.0
        estimated_time = 0.0
        
        if self.is_navigating and self.nav_start_pose:
            if self.planned_path:
                # Calculate remaining distance
                robot_pos = (self.robot_pose['x'], self.robot_pose['y'])
                goal_pos = (self.planned_path[-1]['x'], self.planned_path[-1]['y'])
                remaining_dist = math.sqrt(
                    (goal_pos[0] - robot_pos[0])**2 +
                    (goal_pos[1] - robot_pos[1])**2
                )
                
                # Calculate total distance
                start_pos = (self.nav_start_pose['x'], self.nav_start_pose['y'])
                total_dist = math.sqrt(
                    (goal_pos[0] - start_pos[0])**2 +
                    (goal_pos[1] - start_pos[1])**2
                )
                
                # Estimate time
                if self.velocity > 0.05:
                    estimated_time = remaining_dist / self.velocity
                else:
                    estimated_time = remaining_dist / 0.3
        
        return {
            'is_navigating': self.is_navigating,
            'state': 'Navigating' if self.is_navigating else 'Idle',
            'remaining_distance': remaining_dist,
            'total_distance': total_dist,
            'estimated_time': estimated_time,
            'localization_quality': 95
        }

# Flask app
app = Flask(__name__)
CORS(app)
nav_controller = None

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
        nav_controller.set_command(command)
        return jsonify({'status': 'success', 'command': command})
    
    return jsonify({'status': 'error', 'message': 'Unknown command'})

@app.route('/set_speed', methods=['POST'])
def set_speed():
    data = request.json
    if 'linear' in data:
        nav_controller.linear_speed = float(data['linear'])
    if 'angular' in data:
        nav_controller.angular_speed = float(data['angular'])
    return jsonify({'status': 'success'})

@app.route('/navigate_to_pose', methods=['POST'])
def navigate_to_pose():
    data = request.json
    x = float(data.get('x', 0))
    y = float(data.get('y', 0))
    theta = float(data.get('theta', 0))
    
    if nav_controller.navigate_to_pose(x, y, theta):
        return jsonify({'status': 'success'})
    return jsonify({'status': 'error', 'message': 'Navigation failed'})

@app.route('/goto_location', methods=['POST'])
def goto_location():
    data = request.json
    location_id = data.get('location', '')
    
    if location_id in nav_controller.hospital_locations:
        loc = nav_controller.hospital_locations[location_id]
        if nav_controller.navigate_to_pose(loc['x'], loc['y']):
            return jsonify({'status': 'success', 'location': location_id})
    
    return jsonify({'status': 'error', 'message': 'Location not found'})

@app.route('/cancel_navigation', methods=['POST'])
def cancel_navigation():
    if nav_controller.cancel_navigation():
        return jsonify({'status': 'success'})
    return jsonify({'status': 'error', 'message': 'No active navigation'})

@app.route('/clear_costmaps', methods=['POST'])
def clear_costmaps():
    nav_controller.clear_costmaps()
    return jsonify({'status': 'success'})

@app.route('/save_map', methods=['POST'])
def save_map():
    try:
        maps_dir = os.path.expanduser('~/Last_ros/maps')
        os.makedirs(maps_dir, exist_ok=True)
        
        result = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', f'{maps_dir}/hospital_map'],
            capture_output=True,
            text=True,
            timeout=10
        )
        
        if result.returncode == 0:
            return jsonify({'status': 'success'})
        return jsonify({'status': 'error', 'message': result.stderr})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/get_nav_data', methods=['GET'])
def get_nav_data():
    return jsonify({
        'pose': nav_controller.robot_pose,
        'velocity': nav_controller.velocity,
        'laser': nav_controller.laser_data,
        'planned_path': nav_controller.planned_path,
        'nav_status': nav_controller.get_nav_status(),
        'stats': {
            'distance': nav_controller.distance_traveled,
            'goals_completed': nav_controller.goals_completed
        }
    })

def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)

def main(args=None):
    global nav_controller
    
    rclpy.init(args=args)
    nav_controller = Nav2WebController()

    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    print("\n" + "="*80)
    print("🏥 SYSTÈME DE NAVIGATION HOSPITALIÈRE NAV2")
    print("="*80)
    print(f"📡 Interface Web: http://localhost:5000")
    print("="*80)
    print("✨ Fonctionnalités Nav2:")
    print("   • Navigation autonome avec évitement d'obstacles")
    print("   • Planification de chemin globale")
    print("   • Suivi de trajectoire locale")
    print("   • Relocalisation AMCL")
    print("   • Costmaps dynamiques")
    print("   • 8 salles hospitalières prédéfinies:")
    print("     - 🏢 Réception (-12, 0)")
    print("     - 🚨 Urgences (-11, -8)")
    print("     - 💊 Pharmacie (10, -8)")
    print("     - 🔬 Laboratoire (-1, -9)")
    print("     - 👨‍⚕️ Consultation 1 (-11, 7)")
    print("     - 👩‍⚕️ Consultation 2 (-1, 7)")
    print("     - 🧑‍⚕️ Consultation 3 (10, 7)")
    print("     - 🚪 Entrée (0, 14)")
    print("="*80 + "\n")
    print("🎮 Contrôles:")
    print("   • Mode Manuel: W/A/S/D ou flèches")
    print("   • Mode Auto: Cliquez sur la carte")
    print("   • Boutons: Navigation vers salles")
    print("="*80 + "\n")

    try:
        rclpy.spin(nav_controller)
    except KeyboardInterrupt:
        pass
    finally:
        nav_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()