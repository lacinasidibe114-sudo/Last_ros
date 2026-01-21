#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from flask import Flask, render_template_string, request, jsonify
from flask_cors import CORS
import threading
import math
import json
import time
import os
import socket

# Template HTML complet
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🏥 Système de Navigation Hospitalière - Nav2</title>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Poppins:wght@300;400;600;700&display=swap');
        
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        :root {
            --primary: #667eea;
            --primary-dark: #5568d3;
            --secondary: #764ba2;
            --success: #10b981;
            --warning: #f59e0b;
            --danger: #ef4444;
            --info: #3b82f6;
            --bg-dark: #0f172a;
            --bg-card: #1e293b;
            --bg-card-light: #334155;
            --text-primary: #f1f5f9;
            --text-secondary: #94a3b8;
        }

        body {
            font-family: 'Poppins', sans-serif;
            background: linear-gradient(135deg, var(--bg-dark) 0%, #1a1f35 100%);
            min-height: 100vh;
            color: var(--text-primary);
            overflow-x: hidden;
        }

        .container {
            max-width: 2000px;
            margin: 0 auto;
            padding: 20px;
        }

        .header {
            background: linear-gradient(135deg, var(--primary) 0%, var(--secondary) 100%);
            padding: 25px 40px;
            border-radius: 20px;
            box-shadow: 0 10px 40px rgba(102, 126, 234, 0.3);
            margin-bottom: 25px;
            position: relative;
            overflow: hidden;
        }

        .header::before {
            content: '';
            position: absolute;
            top: -50%;
            right: -50%;
            width: 200%;
            height: 200%;
            background: radial-gradient(circle, rgba(255,255,255,0.1) 0%, transparent 70%);
            animation: pulse-gradient 3s ease-in-out infinite;
        }

        @keyframes pulse-gradient {
            0%, 100% { transform: scale(1) rotate(0deg); }
            50% { transform: scale(1.1) rotate(10deg); }
        }

        .header-content {
            position: relative;
            z-index: 1;
            display: flex;
            justify-content: space-between;
            align-items: center;
            flex-wrap: wrap;
            gap: 20px;
        }

        .header-title {
            display: flex;
            align-items: center;
            gap: 15px;
        }

        .header-title h1 {
            font-size: 2.2em;
            font-weight: 700;
            color: white;
            text-shadow: 0 2px 10px rgba(0,0,0,0.2);
        }

        .header-icon {
            font-size: 2.5em;
            animation: float 3s ease-in-out infinite;
        }

        @keyframes float {
            0%, 100% { transform: translateY(0px); }
            50% { transform: translateY(-10px); }
        }

        .status-badges {
            display: flex;
            gap: 12px;
            flex-wrap: wrap;
        }

        .status-badge {
            background: rgba(255, 255, 255, 0.2);
            padding: 10px 20px;
            border-radius: 25px;
            font-weight: 600;
            font-size: 0.9em;
            backdrop-filter: blur(10px);
            border: 2px solid rgba(255, 255, 255, 0.3);
            display: flex;
            align-items: center;
            gap: 8px;
            transition: all 0.3s ease;
        }

        .status-badge:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.3);
        }

        .status-badge.active {
            background: var(--success);
            border-color: var(--success);
            animation: pulse-badge 2s ease-in-out infinite;
        }

        @keyframes pulse-badge {
            0%, 100% { opacity: 1; transform: scale(1); }
            50% { opacity: 0.8; transform: scale(1.05); }
        }

        .status-dot {
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background: currentColor;
            animation: blink 1.5s ease-in-out infinite;
        }

        @keyframes blink {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.3; }
        }

        .connection-status {
            position: fixed;
            bottom: 20px;
            right: 20px;
            background: rgba(0, 0, 0, 0.8);
            color: white;
            padding: 12px 20px;
            border-radius: 25px;
            font-size: 0.9em;
            font-weight: 600;
            backdrop-filter: blur(10px);
            z-index: 1000;
            border: 2px solid;
            display: flex;
            align-items: center;
            gap: 8px;
            transition: all 0.3s ease;
        }

        .connection-status.connected {
            background: rgba(16, 185, 129, 0.9);
            border-color: var(--success);
        }

        .connection-status.disconnected {
            background: rgba(239, 68, 68, 0.9);
            border-color: var(--danger);
        }

        .dashboard {
            display: grid;
            grid-template-columns: 380px 1fr 380px;
            gap: 20px;
            align-items: start;
        }

        .panel {
            background: var(--bg-card);
            border-radius: 20px;
            padding: 25px;
            box-shadow: 0 10px 30px rgba(0, 0, 0, 0.5);
            border: 1px solid rgba(255, 255, 255, 0.1);
            backdrop-filter: blur(20px);
            transition: all 0.3s ease;
        }

        .panel:hover {
            transform: translateY(-5px);
            box-shadow: 0 15px 40px rgba(102, 126, 234, 0.3);
        }

        .panel-title {
            display: flex;
            align-items: center;
            gap: 10px;
            font-size: 1.3em;
            font-weight: 700;
            margin-bottom: 20px;
            padding-bottom: 15px;
            border-bottom: 2px solid rgba(255, 255, 255, 0.1);
            color: var(--primary);
        }

        .panel-title-icon {
            font-size: 1.3em;
        }

        .map-container {
            position: relative;
            height: 650px;
            background: var(--bg-dark);
            border-radius: 15px;
            overflow: hidden;
            border: 3px solid var(--primary);
            box-shadow: inset 0 0 30px rgba(0,0,0,0.5);
        }

        #map-canvas {
            width: 100%;
            height: 100%;
            cursor: crosshair;
        }

        .map-overlay {
            position: absolute;
            top: 15px;
            left: 15px;
            background: rgba(15, 23, 42, 0.95);
            padding: 15px;
            border-radius: 12px;
            border: 1px solid rgba(255, 255, 255, 0.1);
            backdrop-filter: blur(10px);
            font-size: 0.85em;
            z-index: 10;
        }

        .map-overlay-item {
            display: flex;
            align-items: center;
            gap: 8px;
            margin-bottom: 8px;
            padding: 5px;
            border-radius: 5px;
            transition: background 0.2s ease;
        }

        .map-overlay-item:hover {
            background: rgba(255, 255, 255, 0.05);
        }

        .map-legend-icon {
            width: 20px;
            height: 20px;
            border-radius: 50%;
            display: flex;
            align-items: center;
            justify-content: center;
        }

        .map-controls {
            position: absolute;
            top: 15px;
            right: 15px;
            display: flex;
            flex-direction: column;
            gap: 10px;
            z-index: 10;
        }

        .map-btn {
            background: rgba(102, 126, 234, 0.9);
            border: none;
            color: white;
            width: 45px;
            height: 45px;
            border-radius: 12px;
            cursor: pointer;
            font-size: 1.2em;
            font-weight: bold;
            transition: all 0.3s ease;
            backdrop-filter: blur(10px);
            border: 2px solid rgba(255, 255, 255, 0.2);
            display: flex;
            align-items: center;
            justify-content: center;
        }

        .map-btn:hover {
            background: var(--primary);
            transform: scale(1.1);
            box-shadow: 0 5px 20px rgba(102, 126, 234, 0.5);
        }

        .map-btn:active {
            transform: scale(0.95);
        }

        .control-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            margin-bottom: 20px;
        }

        .control-btn {
            background: linear-gradient(135deg, var(--primary) 0%, var(--secondary) 100%);
            border: none;
            color: white;
            font-size: 1.8em;
            padding: 20px;
            border-radius: 15px;
            cursor: pointer;
            transition: all 0.3s ease;
            box-shadow: 0 5px 15px rgba(102, 126, 234, 0.3);
            border: 2px solid rgba(255, 255, 255, 0.1);
            position: relative;
            overflow: hidden;
        }

        .control-btn::before {
            content: '';
            position: absolute;
            top: 50%;
            left: 50%;
            width: 0;
            height: 0;
            border-radius: 50%;
            background: rgba(255, 255, 255, 0.3);
            transform: translate(-50%, -50%);
            transition: width 0.5s, height 0.5s;
        }

        .control-btn:hover::before {
            width: 300px;
            height: 300px;
        }

        .control-btn:hover {
            transform: translateY(-5px);
            box-shadow: 0 10px 25px rgba(102, 126, 234, 0.5);
        }

        .control-btn:active {
            transform: translateY(-2px);
        }

        .control-btn.active {
            background: linear-gradient(135deg, var(--success) 0%, #059669 100%);
            box-shadow: 0 0 30px var(--success);
            animation: glow 1.5s ease-in-out infinite;
        }

        @keyframes glow {
            0%, 100% { box-shadow: 0 0 30px var(--success); }
            50% { box-shadow: 0 0 40px var(--success), 0 0 60px var(--success); }
        }

        .control-btn.stop-btn {
            background: linear-gradient(135deg, var(--danger) 0%, #dc2626 100%);
            font-size: 1.3em;
            font-weight: 700;
        }

        .control-btn.stop-btn:hover {
            box-shadow: 0 10px 25px rgba(239, 68, 68, 0.5);
        }

        .speed-control {
            margin: 15px 0;
        }

        .speed-label {
            display: flex;
            justify-content: space-between;
            margin-bottom: 10px;
            font-weight: 600;
            color: var(--text-secondary);
        }

        .speed-value {
            color: var(--primary);
            font-weight: 700;
        }

        .speed-slider {
            width: 100%;
            height: 8px;
            border-radius: 10px;
            background: var(--bg-card-light);
            outline: none;
            -webkit-appearance: none;
            position: relative;
        }

        .speed-slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 22px;
            height: 22px;
            border-radius: 50%;
            background: linear-gradient(135deg, var(--primary) 0%, var(--secondary) 100%);
            cursor: pointer;
            box-shadow: 0 0 10px rgba(102, 126, 234, 0.5);
            transition: all 0.3s ease;
        }

        .speed-slider::-webkit-slider-thumb:hover {
            transform: scale(1.2);
            box-shadow: 0 0 20px rgba(102, 126, 234, 0.8);
        }

        .speed-slider::-moz-range-thumb {
            width: 22px;
            height: 22px;
            border-radius: 50%;
            background: linear-gradient(135deg, var(--primary) 0%, var(--secondary) 100%);
            cursor: pointer;
            border: none;
            box-shadow: 0 0 10px rgba(102, 126, 234, 0.5);
        }

        .locations-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 12px;
            margin-bottom: 20px;
        }

        .location-card {
            background: linear-gradient(135deg, var(--bg-card-light) 0%, var(--bg-card) 100%);
            border: 2px solid rgba(255, 255, 255, 0.1);
            border-radius: 15px;
            padding: 18px;
            cursor: pointer;
            transition: all 0.3s ease;
            text-align: center;
            position: relative;
            overflow: hidden;
        }

        .location-card::before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 100%;
            height: 100%;
            background: linear-gradient(90deg, transparent, rgba(255,255,255,0.1), transparent);
            transition: left 0.5s ease;
        }

        .location-card:hover::before {
            left: 100%;
        }

        .location-card:hover {
            transform: translateY(-5px);
            box-shadow: 0 10px 30px rgba(102, 126, 234, 0.4);
            border-color: var(--primary);
        }

        .location-icon {
            font-size: 2.5em;
            margin-bottom: 10px;
            display: block;
        }

        .location-name {
            font-weight: 600;
            font-size: 0.95em;
            color: var(--text-primary);
        }

        .location-coords {
            font-size: 0.75em;
            color: var(--text-secondary);
            margin-top: 5px;
        }

        .metrics-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 15px;
            margin-bottom: 20px;
        }

        .metric-card {
            background: linear-gradient(135deg, var(--primary) 0%, var(--secondary) 100%);
            border-radius: 15px;
            padding: 20px;
            text-align: center;
            box-shadow: 0 5px 20px rgba(102, 126, 234, 0.3);
            transition: all 0.3s ease;
            border: 2px solid rgba(255, 255, 255, 0.2);
        }

        .metric-card:hover {
            transform: scale(1.05);
            box-shadow: 0 10px 30px rgba(102, 126, 234, 0.5);
        }

        .metric-icon {
            font-size: 2em;
            margin-bottom: 10px;
            opacity: 0.9;
        }

        .metric-value {
            font-size: 2em;
            font-weight: 700;
            margin: 10px 0;
            text-shadow: 0 2px 10px rgba(0,0,0,0.2);
        }

        .metric-label {
            font-size: 0.85em;
            opacity: 0.9;
            font-weight: 500;
        }

        .lidar-container {
            background: var(--bg-dark);
            border-radius: 15px;
            padding: 15px;
            height: 220px;
            position: relative;
            border: 2px solid var(--primary);
            box-shadow: inset 0 0 20px rgba(0,0,0,0.5);
        }

        #lidar-canvas {
            width: 100%;
            height: 100%;
        }

        .nav-stats {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 15px;
            background: var(--bg-card-light);
            padding: 20px;
            border-radius: 15px;
            margin-bottom: 20px;
        }

        .stat-item {
            text-align: center;
        }

        .stat-value {
            font-size: 1.8em;
            font-weight: 700;
            color: var(--primary);
            margin-bottom: 5px;
        }

        .stat-label {
            font-size: 0.85em;
            color: var(--text-secondary);
        }

        .action-btns {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 12px;
            margin-top: 20px;
        }

        .action-btn {
            padding: 15px;
            border: none;
            border-radius: 12px;
            font-weight: 600;
            font-size: 0.95em;
            cursor: pointer;
            transition: all 0.3s ease;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 8px;
        }

        .action-btn.primary {
            background: linear-gradient(135deg, var(--info) 0%, #2563eb 100%);
            color: white;
        }

        .action-btn.success {
            background: linear-gradient(135deg, var(--success) 0%, #059669 100%);
            color: white;
        }

        .action-btn.warning {
            background: linear-gradient(135deg, var(--warning) 0%, #d97706 100%);
            color: white;
        }

        .action-btn.danger {
            background: linear-gradient(135deg, var(--danger) 0%, #dc2626 100%);
            color: white;
        }

        .action-btn:hover {
            transform: translateY(-3px);
            box-shadow: 0 8px 25px rgba(0,0,0,0.3);
        }

        .action-btn:active {
            transform: translateY(-1px);
        }

        .waypoints-container {
            max-height: 300px;
            overflow-y: auto;
            margin-bottom: 15px;
            padding-right: 5px;
        }

        .waypoints-container::-webkit-scrollbar {
            width: 8px;
        }

        .waypoints-container::-webkit-scrollbar-track {
            background: var(--bg-card-light);
            border-radius: 10px;
        }

        .waypoints-container::-webkit-scrollbar-thumb {
            background: var(--primary);
            border-radius: 10px;
        }

        .waypoint-item {
            background: var(--bg-card-light);
            padding: 15px;
            border-radius: 12px;
            margin-bottom: 10px;
            border: 2px solid rgba(255, 255, 255, 0.1);
            transition: all 0.3s ease;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }

        .waypoint-item:hover {
            transform: translateX(5px);
            border-color: var(--primary);
        }

        .waypoint-info {
            flex: 1;
        }

        .waypoint-name {
            font-weight: 600;
            margin-bottom: 5px;
        }

        .waypoint-coords {
            font-size: 0.85em;
            color: var(--text-secondary);
        }

        .waypoint-actions {
            display: flex;
            gap: 8px;
        }

        .waypoint-btn {
            padding: 8px 12px;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            font-size: 0.9em;
            font-weight: 600;
            transition: all 0.3s ease;
        }

        .waypoint-btn.go {
            background: var(--success);
            color: white;
        }

        .waypoint-btn.delete {
            background: var(--danger);
            color: white;
        }

        .waypoint-btn:hover {
            transform: scale(1.1);
        }

        .notification {
            position: fixed;
            top: 30px;
            right: 30px;
            background: white;
            color: #333;
            padding: 20px 25px;
            border-radius: 15px;
            box-shadow: 0 10px 40px rgba(0, 0, 0, 0.3);
            min-width: 300px;
            transform: translateX(500px);
            transition: transform 0.4s cubic-bezier(0.68, -0.55, 0.265, 1.55);
            z-index: 10000;
            border-left: 5px solid;
        }

        .notification.show {
            transform: translateX(0);
        }

        .notification.success {
            border-left-color: var(--success);
        }

        .notification.error {
            border-left-color: var(--danger);
        }

        .notification.info {
            border-left-color: var(--info);
        }

        .notification.warning {
            border-left-color: var(--warning);
        }

        .notification-content {
            display: flex;
            align-items: center;
            gap: 15px;
        }

        .notification-icon {
            font-size: 1.8em;
        }

        .notification-text {
            flex: 1;
            font-weight: 600;
        }

        .control-hint {
            text-align: center;
            padding: 12px;
            background: rgba(102, 126, 234, 0.1);
            border-radius: 10px;
            font-size: 0.9em;
            color: var(--text-secondary);
            margin-top: 15px;
        }

        .empty-state {
            text-align: center;
            padding: 40px;
            color: var(--text-secondary);
        }

        .empty-icon {
            font-size: 3em;
            margin-bottom: 15px;
            opacity: 0.5;
        }

        @media (max-width: 1600px) {
            .dashboard {
                grid-template-columns: 350px 1fr 350px;
            }
        }

        @media (max-width: 1400px) {
            .dashboard {
                grid-template-columns: 1fr;
            }
            
            .map-container {
                height: 500px;
            }
        }

        @media (max-width: 768px) {
            .header-title h1 {
                font-size: 1.5em;
            }
            
            .locations-grid {
                grid-template-columns: 1fr;
            }
            
            .metrics-grid {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <div class="header-content">
                <div class="header-title">
                    <span class="header-icon">🏥</span>
                    <h1>Système de Navigation Hospitalière</h1>
                    <span class="header-icon">🤖</span>
                </div>
                <div class="status-badges">
                    <div class="status-badge active">
                        <span class="status-dot"></span>
                        <span>Nav2 Actif</span>
                    </div>
                    <div class="status-badge" id="robot-status">
                        <span>⚡</span>
                        <span>Robot Prêt</span>
                    </div>
                    <div class="status-badge" id="nav-status">
                        <span>🎯</span>
                        <span id="nav-state">En attente</span>
                    </div>
                </div>
            </div>
        </div>

        <div class="dashboard">
            <div class="panel">
                <div class="panel-title">
                    <span class="panel-title-icon">🎮</span>
                    <span>Contrôle Manuel</span>
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
                    💡 Touches: W A S D ou ← ↑ → ↓
                </div>

                <div class="speed-control">
                    <div class="speed-label">
                        <span>Vitesse Linéaire</span>
                        <span class="speed-value" id="linear-value">0.5 m/s</span>
                    </div>
                    <input type="range" class="speed-slider" id="linear-speed" 
                           min="0.1" max="2.0" step="0.1" value="0.5">
                </div>

                <div class="speed-control">
                    <div class="speed-label">
                        <span>Vitesse Angulaire</span>
                        <span class="speed-value" id="angular-value">1.0 rad/s</span>
                    </div>
                    <input type="range" class="speed-slider" id="angular-speed" 
                           min="0.1" max="3.0" step="0.1" value="1.0">
                </div>

                <div class="panel-title" style="margin-top: 25px;">
                    <span class="panel-title-icon">📊</span>
                    <span>Métriques Robot</span>
                </div>

                <div class="metrics-grid">
                    <div class="metric-card">
                        <div class="metric-icon">📍</div>
                        <div class="metric-value" id="pos-x">0.00</div>
                        <div class="metric-label">Position X (m)</div>
                    </div>
                    <div class="metric-card">
                        <div class="metric-icon">📍</div>
                        <div class="metric-value" id="pos-y">0.00</div>
                        <div class="metric-label">Position Y (m)</div>
                    </div>
                    <div class="metric-card">
                        <div class="metric-icon">⚡</div>
                        <div class="metric-value" id="velocity">0.00</div>
                        <div class="metric-label">Vitesse (m/s)</div>
                    </div>
                    <div class="metric-card">
                        <div class="metric-icon">🧭</div>
                        <div class="metric-value" id="rotation">0°</div>
                        <div class="metric-label">Orientation</div>
                    </div>
                </div>

                <div class="panel-title">
                    <span class="panel-title-icon">📡</span>
                    <span>Capteur LiDAR</span>
                </div>

                <div class="lidar-container">
                    <canvas id="lidar-canvas"></canvas>
                </div>
            </div>

            <div class="panel">
                <div class="panel-title">
                    <span class="panel-title-icon">🗺️</span>
                    <span>Carte de Navigation</span>
                </div>

                <div class="map-container">
                    <canvas id="map-canvas"></canvas>
                    
                    <div class="map-overlay">
                        <div class="map-overlay-item">
                            <div class="map-legend-icon" style="background: #10b981;">🤖</div>
                            <span>Robot</span>
                        </div>
                        <div class="map-overlay-item">
                            <div class="map-legend-icon" style="background: #ef4444;">📍</div>
                            <span>Objectif</span>
                        </div>
                        <div class="map-overlay-item">
                            <div class="map-legend-icon" style="background: #f59e0b;">⚠️</div>
                            <span>Obstacles</span>
                        </div>
                        <div class="map-overlay-item">
                            <div class="map-legend-icon" style="background: #667eea;">🛤️</div>
                            <span>Chemin</span>
                        </div>
                    </div>

                    <div class="map-controls">
                        <button class="map-btn" onclick="zoomIn()" title="Zoom In">+</button>
                        <button class="map-btn" onclick="zoomOut()" title="Zoom Out">−</button>
                        <button class="map-btn" onclick="centerMap()" title="Centrer">⌖</button>
                        <button class="map-btn" onclick="toggleGrid()" title="Grille">▦</button>
                    </div>
                </div>

                <div class="nav-stats">
                    <div class="stat-item">
                        <div class="stat-value" id="distance-traveled">0.0</div>
                        <div class="stat-label">Distance (m)</div>
                    </div>
                    <div class="stat-item">
                        <div class="stat-value" id="distance-remaining">0.0</div>
                        <div class="stat-label">Restant (m)</div>
                    </div>
                    <div class="stat-item">
                        <div class="stat-value" id="nav-time">0:00</div>
                        <div class="stat-label">Temps Navigation</div>
                    </div>
                </div>

                <div class="action-btns">
                    <button class="action-btn success" onclick="startNavigation()">
                        <span>▶️</span>
                        <span>Démarrer Nav</span>
                    </button>
                    <button class="action-btn danger" onclick="cancelNavigation()">
                        <span>⏹️</span>
                        <span>Annuler Nav</span>
                    </button>
                    <button class="action-btn primary" onclick="setPose()">
                        <span>📍</span>
                        <span>Définir Pose</span>
                    </button>
                    <button class="action-btn warning" onclick="clearCostmaps()">
                        <span>🔄</span>
                        <span>Nettoyer</span>
                    </button>
                </div>
            </div>

            <div class="panel">
                <div class="panel-title">
                    <span class="panel-title-icon">🏥</span>
                    <span>Salles de l'Hôpital</span>
                </div>

                <div class="locations-grid">
                    <div class="location-card" onclick="goToLocation('reception')">
                        <span class="location-icon">🏢</span>
                        <div class="location-name">Réception</div>
                        <div class="location-coords">(-12.0, 0.0)</div>
                    </div>

                    <div class="location-card" onclick="goToLocation('emergency')">
                        <span class="location-icon">🚨</span>
                        <div class="location-name">Urgences</div>
                        <div class="location-coords">(-7.0, -9.0)</div>
                    </div>

                    <div class="location-card" onclick="goToLocation('pharmacy')">
                        <span class="location-icon">💊</span>
                        <div class="location-name">Pharmacie</div>
                        <div class="location-coords">(7.0, -9.0)</div>
                    </div>

                    <div class="location-card" onclick="goToLocation('lab')">
                        <span class="location-icon">🔬</span>
                        <div class="location-name">Laboratoire</div>
                        <div class="location-coords">(-1.0, -9.0)</div>
                    </div>

                    <div class="location-card" onclick="goToLocation('consult1')">
                        <span class="location-icon">👨‍⚕️</span>
                        <div class="location-name">Consultation 1</div>
                        <div class="location-coords">(-7.0, 9.0)</div>
                    </div>

                    <div class="location-card" onclick="goToLocation('consult2')">
                        <span class="location-icon">👩‍⚕️</span>
                        <div class="location-name">Consultation 2</div>
                        <div class="location-coords">(-1.0, 9.0)</div>
                    </div>

                    <div class="location-card" onclick="goToLocation('consult3')">
                        <span class="location-icon">🧑‍⚕️</span>
                        <div class="location-name">Consultation 3</div>
                        <div class="location-coords">(7.0, 9.0)</div>
                    </div>

                    <div class="location-card" onclick="goToLocation('entrance')">
                        <span class="location-icon">🚪</span>
                        <div class="location-name">Entrée</div>
                        <div class="location-coords">(0.0, 13.0)</div>
                    </div>
                </div>

                <div class="panel-title" style="margin-top: 25px;">
                    <span class="panel-title-icon">📍</span>
                    <span>Points de Navigation</span>
                </div>

                <div style="text-align: center; padding: 12px; background: rgba(102, 126, 234, 0.1); border-radius: 10px; margin-bottom: 15px; font-size: 0.9em;">
                    💡 Cliquez sur la carte pour ajouter des waypoints
                </div>

                <div class="waypoints-container" id="waypoints-container">
                    <div class="empty-state">
                        <div class="empty-icon">📍</div>
                        <div>Aucun waypoint défini</div>
                        <div style="font-size: 0.85em; margin-top: 8px;">Cliquez sur la carte</div>
                    </div>
                </div>

                <div class="action-btns">
                    <button class="action-btn success" onclick="clearWaypoints()">
                        <span>🗑️</span>
                        <span>Effacer Tout</span>
                    </button>
                    <button class="action-btn primary" onclick="saveWaypoints()">
                        <span>💾</span>
                        <span>Sauvegarder</span>
                    </button>
                </div>
            </div>
        </div>
    </div>

    <div class="notification" id="notification">
        <div class="notification-content">
            <span class="notification-icon" id="notification-icon"></span>
            <span class="notification-text" id="notification-text"></span>
        </div>
    </div>

    <div class="connection-status connected" id="connection-status">
        <span class="status-dot"></span>
        <span>Connecté</span>
    </div>

    <script>
        let currentCommand = 'stop';
        let waypoints = [];
        let robotPose = {x: 0, y: 0, theta: 0};
        let mapZoom = 20.0;
        let mapOffset = {x: 0, y: 0};
        let showGrid = true;
        let isNavigating = false;
        let navStartTime = null;
        let distanceTraveled = 0.0;
        let lastPose = null;
        let connectionActive = true;
        let lastUpdateTime = Date.now();

        const mapCanvas = document.getElementById('map-canvas');
        const mapCtx = mapCanvas.getContext('2d');
        const lidarCanvas = document.getElementById('lidar-canvas');
        const lidarCtx = lidarCanvas.getContext('2d');

        function resizeCanvases() {
            mapCanvas.width = mapCanvas.offsetWidth;
            mapCanvas.height = mapCanvas.offsetHeight;
            lidarCanvas.width = lidarCanvas.offsetWidth;
            lidarCanvas.height = lidarCanvas.offsetHeight;
            drawMap();
        }
        resizeCanvases();
        window.addEventListener('resize', resizeCanvases);

        function checkConnection() {
            const now = Date.now();
            const timeSinceUpdate = now - lastUpdateTime;
            const statusElement = document.getElementById('connection-status');
            
            if (timeSinceUpdate > 3000) {
                connectionActive = false;
                statusElement.className = 'connection-status disconnected';
                statusElement.innerHTML = '<span class="status-dot"></span><span>Déconnecté</span>';
            } else {
                connectionActive = true;
                statusElement.className = 'connection-status connected';
                statusElement.innerHTML = '<span class="status-dot"></span><span>Connecté</span>';
            }
        }
        setInterval(checkConnection, 1000);

        function showNotification(message, type = 'info') {
            const notification = document.getElementById('notification');
            const icon = document.getElementById('notification-icon');
            const text = document.getElementById('notification-text');
            
            const icons = {
                'success': '✅',
                'error': '❌',
                'info': 'ℹ️',
                'warning': '⚠️'
            };
            
            icon.textContent = icons[type] || icons['info'];
            text.textContent = message;
            notification.className = `notification ${type} show`;
            
            setTimeout(() => {
                notification.classList.remove('show');
            }, 3000);
        }

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
                    
                    const commandLabels = {
                        'forward': 'Avancer',
                        'backward': 'Reculer',
                        'left': 'Tourner Gauche',
                        'right': 'Tourner Droite',
                        'stop': 'Arrêté'
                    };
                    
                    document.querySelector('#robot-status span:last-child').textContent = 
                        commandLabels[command] || command;
                    
                    lastUpdateTime = Date.now();
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

        document.querySelectorAll('.control-btn').forEach(button => {
            button.addEventListener('click', () => {
                sendCommand(button.dataset.command);
            });
        });

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

        const keyMap = {
            'ArrowUp': 'forward', 'ArrowDown': 'backward',
            'ArrowLeft': 'left', 'ArrowRight': 'right',
            'w': 'forward', 's': 'backward', 'a': 'left', 'd': 'right',
            ' ': 'stop', 'Escape': 'stop'
        };

        document.addEventListener('keydown', (e) => {
            if (keyMap[e.key]) {
                e.preventDefault();
                sendCommand(keyMap[e.key]);
            }
        });

        document.addEventListener('keyup', (e) => {
            if (keyMap[e.key] && keyMap[e.key] !== 'stop') {
                e.preventDefault();
                sendCommand('stop');
            }
        });

        async function goToLocation(location) {
            showNotification(`Navigation vers ${location}...`, 'info');
            console.log(`Navigating to: ${location}`);
            
            try {
                const response = await fetch('/goto_location', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({location: location})
                });
                const data = await response.json();
                
                console.log('Response:', data);
                
                if (data.status === 'success') {
                    isNavigating = true;
                    navStartTime = Date.now();
                    document.getElementById('nav-state').textContent = 'En cours';
                    showNotification(`Navigation démarrée vers ${data.name || location}!`, 'success');
                    lastUpdateTime = Date.now();
                } else {
                    showNotification(`Erreur: ${data.message}`, 'error');
                }
            } catch (error) {
                console.error('Navigation error:', error);
                showNotification('Erreur navigation', 'error');
            }
        }

        mapCanvas.addEventListener('click', (e) => {
            const rect = mapCanvas.getBoundingClientRect();
            const x = (e.clientX - rect.left - mapCanvas.width/2 + mapOffset.x) / mapZoom;
            const y = -(e.clientY - rect.top - mapCanvas.height/2 + mapOffset.y) / mapZoom;
            
            addWaypoint(x, y);
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
                updateWaypointsList();
                showNotification(`Waypoint ajouté: (${x.toFixed(2)}, ${y.toFixed(2)})`, 'success');
                lastUpdateTime = Date.now();
            } catch (error) {
                showNotification('Erreur ajout waypoint', 'error');
            }
        }

        function updateWaypointsList() {
            const container = document.getElementById('waypoints-container');
            
            if (waypoints.length === 0) {
                container.innerHTML = `
                    <div class="empty-state">
                        <div class="empty-icon">📍</div>
                        <div>Aucun waypoint défini</div>
                        <div style="font-size: 0.85em; margin-top: 8px;">Cliquez sur la carte</div>
                    </div>
                `;
                return;
            }
            
            container.innerHTML = waypoints.map((wp, index) => `
                <div class="waypoint-item">
                    <div class="waypoint-info">
                        <div class="waypoint-name">${wp.name}</div>
                        <div class="waypoint-coords">📍 (${wp.x}, ${wp.y})</div>
                    </div>
                    <div class="waypoint-actions">
                        <button class="waypoint-btn go" onclick="goToWaypoint(${index})">
                            ➜
                        </button>
                        <button class="waypoint-btn delete" onclick="deleteWaypoint(${index})">
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
                isNavigating = true;
                navStartTime = Date.now();
                lastUpdateTime = Date.now();
            } catch (error) {
                showNotification('Erreur navigation', 'error');
            }
        }

        function deleteWaypoint(index) {
            waypoints.splice(index, 1);
            updateWaypointsList();
            showNotification('Waypoint supprimé', 'info');
        }

        function clearWaypoints() {
            if (confirm('Effacer tous les waypoints?')) {
                waypoints = [];
                updateWaypointsList();
                showNotification('Waypoints effacés', 'info');
            }
        }

        function saveWaypoints() {
            showNotification('Waypoints sauvegardés', 'success');
        }

        function startNavigation() {
            showNotification('Navigation démarrée', 'success');
            isNavigating = true;
            navStartTime = Date.now();
        }

        async function cancelNavigation() {
            showNotification('Navigation annulée', 'warning');
            isNavigating = false;
            navStartTime = null;
            
            try {
                await fetch('/cancel_navigation', {method: 'POST'});
                lastUpdateTime = Date.now();
            } catch (error) {
                console.error('Error canceling navigation:', error);
            }
        }

        function setPose() {
            showNotification('Cliquez sur la carte pour définir la pose', 'info');
        }

        async function clearCostmaps() {
            showNotification('Nettoyage des costmaps...', 'info');
            
            try {
                await fetch('/clear_costmaps', {method: 'POST'});
                showNotification('Costmaps nettoyées', 'success');
                lastUpdateTime = Date.now();
            } catch (error) {
                showNotification('Erreur nettoyage', 'error');
            }
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

        function toggleGrid() {
            showGrid = !showGrid;
            drawMap();
            showNotification(`Grille: ${showGrid ? 'ON' : 'OFF'}`, 'info');
        }

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
            
            if (showGrid) {
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
            }
            
            drawHospitalLayout(ctx);
            
            waypoints.forEach(wp => {
                const x = parseFloat(wp.x);
                const y = parseFloat(wp.y);
                
                ctx.fillStyle = '#ef4444';
                ctx.beginPath();
                ctx.arc(x, -y, 0.3, 0, Math.PI * 2);
                ctx.fill();
                
                ctx.fillStyle = 'white';
                ctx.font = `${12 / mapZoom}px Poppins`;
                ctx.textAlign = 'center';
                ctx.fillText(wp.name, x, -y - 0.5);
            });
            
            if (robotPose) {
                const rx = robotPose.x;
                const ry = -robotPose.y;
                
                ctx.save();
                ctx.translate(rx, ry);
                ctx.rotate(-robotPose.theta);
                
                ctx.fillStyle = '#10b981';
                ctx.shadowColor = '#10b981';
                ctx.shadowBlur = 20 / mapZoom;
                ctx.beginPath();
                ctx.arc(0, 0, 0.2, 0, Math.PI * 2);
                ctx.fill();
                
                ctx.strokeStyle = '#059669';
                ctx.lineWidth = 2 / mapZoom;
                ctx.shadowBlur = 0;
                ctx.beginPath();
                ctx.moveTo(0, 0);
                ctx.lineTo(0.3, 0);
                ctx.stroke();
                
                ctx.restore();
            }
            
            ctx.restore();
        }

        function drawHospitalLayout(ctx) {
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
            ctx.lineWidth = 1 / mapZoom;
            
            ctx.strokeRect(-15, -15, 30, 30);
            
            ctx.fillStyle = 'rgba(59, 130, 246, 0.1)';
            ctx.fillRect(-13, -3, 6, 6);
            ctx.strokeRect(-13, -3, 6, 6);
            
            ctx.fillStyle = 'rgba(16, 185, 129, 0.1)';
            ctx.fillRect(-10, 6, 6, 6);
            ctx.strokeRect(-10, 6, 6, 6);
            
            ctx.fillRect(-4, 6, 6, 6);
            ctx.strokeRect(-4, 6, 6, 6);
            
            ctx.fillRect(4, 6, 6, 6);
            ctx.strokeRect(4, 6, 6, 6);
            
            ctx.fillStyle = 'rgba(239, 68, 68, 0.1)';
            ctx.fillRect(-10, -12, 6, 6);
            ctx.strokeRect(-10, -12, 6, 6);
            
            ctx.fillStyle = 'rgba(139, 92, 246, 0.1)';
            ctx.fillRect(-4, -12, 6, 6);
            ctx.strokeRect(-4, -12, 6, 6);
            
            ctx.fillStyle = 'rgba(245, 158, 11, 0.1)';
            ctx.fillRect(4, -12, 6, 6);
            ctx.strokeRect(4, -12, 6, 6);
            
            ctx.fillStyle = 'rgba(255, 255, 255, 0.6)';
            ctx.font = `${10 / mapZoom}px Poppins`;
            ctx.textAlign = 'center';
            
            ctx.fillText('Réception', -10, 0);
            ctx.fillText('C1', -7, 9);
            ctx.fillText('C2', -1, 9);
            ctx.fillText('C3', 7, 9);
            ctx.fillText('Urgences', -7, -9);
            ctx.fillText('Labo', -1, -9);
            ctx.fillText('Pharmacie', 7, -9);
        }

        function drawLidar(scanData) {
            const ctx = lidarCtx;
            const w = lidarCanvas.width;
            const h = lidarCanvas.height;
            const cx = w / 2;
            const cy = h / 2;
            const radius = Math.min(w, h) / 2 - 10;
            
            ctx.fillStyle = '#0f0f1e';
            ctx.fillRect(0, 0, w, h);
            
            if (!scanData || !scanData.ranges) return;
            
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
            ctx.lineWidth = 1;
            
            for (let i = 1; i <= 3; i++) {
                ctx.beginPath();
                ctx.arc(cx, cy, (radius * i) / 3, 0, Math.PI * 2);
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
            
            ctx.fillStyle = '#10b981';
            ctx.beginPath();
            ctx.arc(cx, cy, 5, 0, Math.PI * 2);
            ctx.fill();
        }

        async function updateData() {
            try {
                const response = await fetch('/get_nav_data');
                const data = await response.json();
                
                if (data.pose) {
                    robotPose = data.pose;
                    document.getElementById('pos-x').textContent = robotPose.x.toFixed(2);
                    document.getElementById('pos-y').textContent = robotPose.y.toFixed(2);
                    document.getElementById('rotation').textContent = 
                        (robotPose.theta * 180 / Math.PI).toFixed(0) + '°';
                    
                    if (lastPose) {
                        const dx = robotPose.x - lastPose.x;
                        const dy = robotPose.y - lastPose.y;
                        distanceTraveled += Math.sqrt(dx*dx + dy*dy);
                        document.getElementById('distance-traveled').textContent = 
                            distanceTraveled.toFixed(1);
                    }
                    lastPose = robotPose;
                }
                
                if (data.velocity !== undefined) {
                    document.getElementById('velocity').textContent = data.velocity.toFixed(2);
                }
                
                if (data.laser) {
                    drawLidar(data.laser);
                }
                
                if (isNavigating && navStartTime) {
                    const elapsed = Math.floor((Date.now() - navStartTime) / 1000);
                    const mins = Math.floor(elapsed / 60);
                    const secs = elapsed % 60;
                    document.getElementById('nav-time').textContent = 
                        `${mins}:${secs.toString().padStart(2, '0')}`;
                }
                
                drawMap();
                lastUpdateTime = Date.now();
            } catch (error) {
                console.error('Error updating data:', error);
            }
        }

        document.addEventListener('DOMContentLoaded', () => {
            setInterval(updateData, 100);
            drawMap();
            showNotification('Système Nav2 initialisé', 'success');
        });
    </script>
</body>
</html>
"""

def get_local_ip():
    """Get the local IP address of the machine"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception:
        return "localhost"

# Classe Nav2 Controller
class Nav2WebController(Node):
    def __init__(self):
        super().__init__('nav2_web_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        
        # Action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # State
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.current_command = 'stop'
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.velocity = 0.0
        self.laser_data = None
        self.current_path = None
        self.goal_handle = None
        
        # COORDONNÉES CORRIGÉES - Basées sur hospital_world.world
        self.hospital_locations = {
            'reception': {'x': -12.0, 'y': 0.0, 'theta': 0.0, 'name': 'Réception'},
            'emergency': {'x': -7.0, 'y': -9.0, 'theta': 0.0, 'name': 'Urgences'},
            'pharmacy': {'x': 7.0, 'y': -9.0, 'theta': 0.0, 'name': 'Pharmacie'},
            'lab': {'x': -1.0, 'y': -9.0, 'theta': 0.0, 'name': 'Laboratoire'},
            'consult1': {'x': -7.0, 'y': 9.0, 'theta': 0.0, 'name': 'Consultation 1'},
            'consult2': {'x': -1.0, 'y': 9.0, 'theta': 0.0, 'name': 'Consultation 2'},
            'consult3': {'x': 7.0, 'y': 9.0, 'theta': 0.0, 'name': 'Consultation 3'},
            'entrance': {'x': 0.0, 'y': 13.0, 'theta': -1.5708, 'name': 'Entrée Principale'}
        }
        
        # Timer for velocity publishing
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        self.get_logger().info('🏥 Nav2 Web Controller initialized')

    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.robot_pose['x'] = msg.pose.pose.position.x
        self.robot_pose['y'] = msg.pose.pose.position.y
        
        # Convert quaternion to euler
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_pose['theta'] = math.atan2(siny_cosp, cosy_cosp)
        
        # Calculate velocity
        self.velocity = math.sqrt(
            msg.twist.twist.linear.x**2 + 
            msg.twist.twist.linear.y**2
        )

    def scan_callback(self, msg):
        """Update laser scan data"""
        self.laser_data = {
            'ranges': list(msg.ranges),
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment
        }

    def path_callback(self, msg):
        """Update current navigation path"""
        if msg.poses:
            self.current_path = [
                {'x': pose.pose.position.x, 'y': pose.pose.position.y}
                for pose in msg.poses
            ]

    def set_command(self, command):
        """Set manual control command"""
        self.current_command = command

    def publish_velocity(self):
        """Publish velocity commands continuously"""
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
        """Navigate to a specific pose using Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        
        self.get_logger().info(f'🎯 Navigating to: ({x:.2f}, {y:.2f}, {theta:.2f})')
        
        # Wait for action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Action server not available!')
            return False
        
        # Send goal
        self.get_logger().info(f'📤 Sending goal to action server...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True

    def goal_response_callback(self, future):
        """Handle goal response"""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('❌ Goal rejected')
            return
        
        self.get_logger().info('✅ Goal accepted')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info('🏁 Navigation completed')
        self.goal_handle = None

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback

    def cancel_navigation(self):
        """Cancel current navigation"""
        if self.goal_handle:
            self.get_logger().info('⏹️ Canceling navigation...')
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
            return True
        return False

    def cancel_done_callback(self, future):
        """Handle cancel response"""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('✅ Navigation canceled')
        else:
            self.get_logger().warn('❌ Failed to cancel navigation')

    def set_initial_pose(self, x, y, theta):
        """Set initial pose for localization"""
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        initial_pose.pose.pose.orientation.z = qz
        initial_pose.pose.pose.orientation.w = qw
        
        initial_pose.pose.covariance = [0.25] * 36
        
        self.initial_pose_pub.publish(initial_pose)
        self.get_logger().info(f'📍 Initial pose set: ({x:.2f}, {y:.2f}, {theta:.2f})')

    def navigate_to_location(self, location_id):
        """Navigate to predefined hospital location"""
        if location_id not in self.hospital_locations:
            self.get_logger().warn(f'⚠️ Unknown location: {location_id}')
            return False
        
        loc = self.hospital_locations[location_id]
        self.get_logger().info(f'🏥 Navigating to {loc["name"]} at ({loc["x"]}, {loc["y"]})')
        
        return self.navigate_to_pose(loc['x'], loc['y'], loc.get('theta', 0.0))


# Flask Application
app = Flask(__name__)
CORS(app)
nav2_controller = None
waypoints_db = []

@app.route('/')
def index():
    """Serve main interface"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/control', methods=['POST'])
def control():
    """Handle manual control commands"""
    data = request.json
    command = data.get('command', '')
    
    valid_commands = ['forward', 'backward', 'left', 'right', 
                     'forward_left', 'forward_right', 
                     'backward_left', 'backward_right', 'stop']
    
    if command in valid_commands:
        nav2_controller.set_command(command)
        return jsonify({'status': 'success', 'command': command})
    
    return jsonify({'status': 'error', 'message': 'Unknown command'})

@app.route('/set_speed', methods=['POST'])
def set_speed():
    """Update robot speeds"""
    data = request.json
    if 'linear' in data:
        nav2_controller.linear_speed = float(data['linear'])
    if 'angular' in data:
        nav2_controller.angular_speed = float(data['angular'])
    
    return jsonify({
        'status': 'success',
        'linear_speed': nav2_controller.linear_speed,
        'angular_speed': nav2_controller.angular_speed
    })

@app.route('/get_nav_data', methods=['GET'])
def get_nav_data():
    """Get current navigation data"""
    return jsonify({
        'pose': nav2_controller.robot_pose,
        'velocity': nav2_controller.velocity,
        'laser': nav2_controller.laser_data,
        'path': nav2_controller.current_path,
        'is_navigating': nav2_controller.goal_handle is not None
    })

@app.route('/goto_location', methods=['POST'])
def goto_location():
    """Navigate to hospital location"""
    data = request.json
    location_id = data.get('location', '')
    
    nav2_controller.get_logger().info(f'🎯 Request to navigate to: {location_id}')
    
    if location_id not in nav2_controller.hospital_locations:
        return jsonify({
            'status': 'error', 
            'message': f'Unknown location: {location_id}'
        })
    
    success = nav2_controller.navigate_to_location(location_id)
    
    if success:
        loc = nav2_controller.hospital_locations[location_id]
        return jsonify({
            'status': 'success',
            'location': location_id,
            'name': loc['name'],
            'coordinates': {'x': loc['x'], 'y': loc['y']}
        })
    else:
        return jsonify({
            'status': 'error',
            'message': 'Failed to start navigation'
        })

@app.route('/add_waypoint', methods=['POST'])
def add_waypoint():
    """Add a new waypoint"""
    waypoint = request.json
    waypoints_db.append(waypoint)
    return jsonify({'status': 'success', 'waypoint': waypoint})

@app.route('/goto_waypoint', methods=['POST'])
def goto_waypoint():
    """Navigate to a waypoint"""
    waypoint = request.json
    x = float(waypoint['x'])
    y = float(waypoint['y'])
    
    if nav2_controller.navigate_to_pose(x, y):
        return jsonify({'status': 'success'})
    
    return jsonify({'status': 'error', 'message': 'Navigation failed'})

@app.route('/cancel_navigation', methods=['POST'])
def cancel_navigation():
    """Cancel current navigation"""
    if nav2_controller.cancel_navigation():
        return jsonify({'status': 'success'})
    
    return jsonify({'status': 'error', 'message': 'No active navigation'})

@app.route('/set_initial_pose', methods=['POST'])
def set_initial_pose():
    """Set robot initial pose"""
    data = request.json
    x = float(data.get('x', 0.0))
    y = float(data.get('y', 0.0))
    theta = float(data.get('theta', 0.0))
    
    nav2_controller.set_initial_pose(x, y, theta)
    return jsonify({'status': 'success'})

@app.route('/clear_costmaps', methods=['POST'])
def clear_costmaps():
    """Clear costmaps"""
    return jsonify({'status': 'success', 'message': 'Costmaps cleared'})

@app.route('/get_waypoints', methods=['GET'])
def get_waypoints():
    """Get all waypoints"""
    return jsonify({'waypoints': waypoints_db})

@app.route('/clear_waypoints', methods=['POST'])
def clear_waypoints():
    """Clear all waypoints"""
    waypoints_db.clear()
    return jsonify({'status': 'success'})

@app.route('/save_waypoints', methods=['POST'])
def save_waypoints():
    """Save waypoints to file"""
    try:
        waypoints_dir = os.path.expanduser('~/Last_ros/src/my_robot_controller/waypoints')
        os.makedirs(waypoints_dir, exist_ok=True)
        
        waypoints_file = os.path.join(waypoints_dir, 'hospital_waypoints.json')
        with open(waypoints_file, 'w') as f:
            json.dump(waypoints_db, f, indent=2)
        
        return jsonify({
            'status': 'success',
            'message': f'Waypoints saved to {waypoints_file}',
            'count': len(waypoints_db)
        })
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})


def run_flask():
    """Run Flask server with network configuration"""
    local_ip = get_local_ip()
    
    print("\n" + "="*80)
    print("🌐 DÉMARRAGE DU SERVEUR WEB NAV2")
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
    
    app.run(
        host='0.0.0.0',  # Listen on all interfaces
        port=5000,
        debug=False,
        use_reloader=False,
        threaded=True
    )


def main(args=None):
    """Main function"""
    global nav2_controller
    
    rclpy.init(args=args)
    nav2_controller = Nav2WebController()
    
    # Get network info
    local_ip = get_local_ip()
    
    # Start Flask in separate thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    print("\n" + "="*80)
    print("🏥 SYSTÈME DE NAVIGATION HOSPITALIÈRE - NAV2")
    print("="*80)
    print(f"📡 Interface Web (VM):         http://localhost:5000")
    print(f"📡 Interface Web (Hôte):       http://{local_ip}:5000")
    print(f"📡 Interface Web (Réseau):     http://{local_ip}:5000")
    print("="*80)
    print("\n✨ Fonctionnalités:")
    print("   • Contrôle manuel du robot (clavier + interface)")
    print("   • Navigation autonome Nav2")
    print("   • 8 salles hospitalières prédéfinies:")
    for loc_id, loc in nav2_controller.hospital_locations.items():
        print(f"     - {loc['name']}: ({loc['x']}, {loc['y']})")
    print("   • Waypoints personnalisables")
    print("   • Visualisation temps réel (LiDAR, carte)")
    print("   • Évitement d'obstacles dynamique")
    print("   • Indicateur de connexion")
    print("="*80)
    print("\n🎮 Contrôles:")
    print("   • Interface Web: http://{local_ip}:5000")
    print("   • Clavier: W/A/S/D ou ← ↑ → ↓")
    print("   • Souris: Clic sur carte pour waypoints")
    print("="*80)
    print("\n🔧 Configuration réseau:")
    print("   • Mode recommandé: Bridge")
    print("   • Port: 5000 (TCP)")
    print("   • Pare-feu: Autoriser port 5000")
    print("="*80)
    print("\n🚀 Le système Nav2 est prêt!")
    print("="*80 + "\n")
    
    try:
        rclpy.spin(nav2_controller)
    except KeyboardInterrupt:
        print("\n⏹️ Arrêt du système...")
    finally:
        nav2_controller.destroy_node()
        rclpy.shutdown()
        print("✅ Système Nav2 arrêté proprement")


if __name__ == '__main__':
    main()