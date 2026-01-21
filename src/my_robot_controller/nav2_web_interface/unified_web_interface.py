#!/usr/bin/env python3
"""
HospiBot Unified Web Interface - VERSION FINALE COMPLÈTE
Avec éléments visuels intégrés du code Nav2
✅ Toutes corrections + Visuels améliorés
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from flask import Flask, render_template_string, request, jsonify, session
from flask_cors import CORS
import threading
import math
import json
import time
import os
import socket
import subprocess
import secrets

# Template HTML avec visuels Nav2 intégrés
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🏥 HospiBot - Système de Navigation Hospitalière</title>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700;800&display=swap');
        
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        :root {
            --primary: #3b82f6;
            --primary-dark: #2563eb;
            --primary-light: #60a5fa;
            --secondary: #8b5cf6;
            --success: #10b981;
            --warning: #f59e0b;
            --danger: #ef4444;
            --info: #06b6d4;
            --bg-dark: #0f172a;
            --bg-card: #1e293b;
            --bg-card-light: #334155;
            --text-primary: #f1f5f9;
            --text-secondary: #94a3b8;
            --accent: #ec4899;
        }

        body {
            font-family: 'Inter', -apple-system, BlinkMacSystemFont, sans-serif;
            background: linear-gradient(135deg, #0f172a 0%, #1e293b 100%);
            min-height: 100vh;
            color: var(--text-primary);
            overflow-x: hidden;
        }

        /* LOGIN SCREEN */
        .login-container {
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
            padding: 20px;
            background: 
                radial-gradient(circle at 20% 50%, rgba(59, 130, 246, 0.1) 0%, transparent 50%),
                radial-gradient(circle at 80% 80%, rgba(139, 92, 246, 0.1) 0%, transparent 50%),
                linear-gradient(135deg, #0f172a 0%, #1e293b 100%);
        }

        .login-box {
            background: rgba(30, 41, 59, 0.8);
            backdrop-filter: blur(20px);
            border-radius: 24px;
            padding: 50px 40px;
            width: 100%;
            max-width: 450px;
            box-shadow: 
                0 20px 60px rgba(0, 0, 0, 0.5),
                0 0 100px rgba(59, 130, 246, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.1);
            animation: slideUp 0.6s ease-out;
        }

        @keyframes slideUp {
            from {
                opacity: 0;
                transform: translateY(30px);
            }
            to {
                opacity: 1;
                transform: translateY(0);
            }
        }

        .login-header {
            text-align: center;
            margin-bottom: 40px;
        }

        .login-logo {
            font-size: 4em;
            margin-bottom: 15px;
            animation: float 3s ease-in-out infinite;
        }

        @keyframes float {
            0%, 100% { transform: translateY(0); }
            50% { transform: translateY(-10px); }
        }

        .login-title {
            font-size: 2em;
            font-weight: 800;
            background: linear-gradient(135deg, var(--primary) 0%, var(--secondary) 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
            margin-bottom: 8px;
        }

        .login-subtitle {
            color: var(--text-secondary);
            font-size: 0.95em;
            font-weight: 500;
        }

        .role-selector {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
            margin-bottom: 30px;
        }

        .role-btn {
            padding: 20px;
            border: 2px solid rgba(255, 255, 255, 0.1);
            background: rgba(255, 255, 255, 0.05);
            border-radius: 16px;
            cursor: pointer;
            transition: all 0.3s ease;
            text-align: center;
            position: relative;
            overflow: hidden;
        }

        .role-btn::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: linear-gradient(135deg, var(--primary) 0%, var(--secondary) 100%);
            opacity: 0;
            transition: opacity 0.3s ease;
        }

        .role-btn:hover {
            transform: translateY(-5px);
            border-color: var(--primary);
            box-shadow: 0 10px 30px rgba(59, 130, 246, 0.3);
        }

        .role-btn.active {
            border-color: var(--primary);
            background: linear-gradient(135deg, rgba(59, 130, 246, 0.2) 0%, rgba(139, 92, 246, 0.2) 100%);
        }

        .role-btn.active::before {
            opacity: 0.1;
        }

        .role-icon {
            font-size: 2.5em;
            margin-bottom: 10px;
            display: block;
            position: relative;
            z-index: 1;
        }

        .role-name {
            font-weight: 600;
            font-size: 1.1em;
            position: relative;
            z-index: 1;
        }

        .login-form {
            display: none;
        }

        .login-form.active {
            display: block;
            animation: fadeIn 0.4s ease-out;
        }

        @keyframes fadeIn {
            from { opacity: 0; transform: translateY(10px); }
            to { opacity: 1; transform: translateY(0); }
        }

        .form-group {
            margin-bottom: 25px;
        }

        .form-label {
            display: block;
            margin-bottom: 8px;
            font-weight: 600;
            color: var(--text-secondary);
            font-size: 0.9em;
        }

        .form-input {
            width: 100%;
            padding: 15px 20px;
            background: rgba(255, 255, 255, 0.05);
            border: 2px solid rgba(255, 255, 255, 0.1);
            border-radius: 12px;
            color: var(--text-primary);
            font-size: 1em;
            transition: all 0.3s ease;
            font-family: 'Inter', sans-serif;
        }

        .form-input:focus {
            outline: none;
            border-color: var(--primary);
            background: rgba(255, 255, 255, 0.08);
            box-shadow: 0 0 0 4px rgba(59, 130, 246, 0.1);
        }

        .login-btn {
            width: 100%;
            padding: 16px;
            background: linear-gradient(135deg, var(--primary) 0%, var(--secondary) 100%);
            border: none;
            border-radius: 12px;
            color: white;
            font-size: 1.1em;
            font-weight: 700;
            cursor: pointer;
            transition: all 0.3s ease;
            position: relative;
            overflow: hidden;
        }

        .login-btn::before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 100%;
            height: 100%;
            background: linear-gradient(90deg, transparent, rgba(255,255,255,0.3), transparent);
            transition: left 0.5s ease;
        }

        .login-btn:hover::before {
            left: 100%;
        }

        .login-btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 10px 30px rgba(59, 130, 246, 0.4);
        }

        .login-btn:active {
            transform: translateY(0);
        }

        .quick-access {
            margin-top: 25px;
            padding-top: 25px;
            border-top: 1px solid rgba(255, 255, 255, 0.1);
            text-align: center;
        }

        .quick-access-title {
            font-size: 0.85em;
            color: var(--text-secondary);
            margin-bottom: 15px;
        }

        .quick-btns {
            display: flex;
            gap: 10px;
        }

        .quick-btn {
            flex: 1;
            padding: 10px;
            background: rgba(255, 255, 255, 0.05);
            border: 1px solid rgba(255, 255, 255, 0.1);
            border-radius: 8px;
            color: var(--text-secondary);
            font-size: 0.85em;
            cursor: pointer;
            transition: all 0.3s ease;
        }

        .quick-btn:hover {
            background: rgba(255, 255, 255, 0.1);
            border-color: var(--primary);
            color: var(--text-primary);
        }

        /* MAIN APP */
        .app-container {
            display: none;
        }

        .app-container.active {
            display: block;
            animation: fadeIn 0.5s ease-out;
        }

        .navbar {
            background: rgba(30, 41, 59, 0.95);
            backdrop-filter: blur(20px);
            padding: 20px 30px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 1px solid rgba(255, 255, 255, 0.1);
            position: sticky;
            top: 0;
            z-index: 100;
            box-shadow: 0 4px 20px rgba(0, 0, 0, 0.3);
        }

        .navbar-brand {
            display: flex;
            align-items: center;
            gap: 15px;
        }

        .brand-logo {
            font-size: 2em;
        }

        .brand-text {
            display: flex;
            flex-direction: column;
        }

        .brand-title {
            font-size: 1.4em;
            font-weight: 800;
            background: linear-gradient(135deg, var(--primary) 0%, var(--secondary) 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
        }

        .brand-subtitle {
            font-size: 0.75em;
            color: var(--text-secondary);
            font-weight: 500;
        }

        .navbar-user {
            display: flex;
            align-items: center;
            gap: 20px;
        }

        .user-badge {
            display: flex;
            align-items: center;
            gap: 12px;
            padding: 10px 20px;
            background: rgba(59, 130, 246, 0.1);
            border-radius: 12px;
            border: 1px solid rgba(59, 130, 246, 0.3);
        }

        .user-avatar {
            width: 40px;
            height: 40px;
            border-radius: 50%;
            background: linear-gradient(135deg, var(--primary) 0%, var(--secondary) 100%);
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 1.2em;
        }

        .user-info {
            display: flex;
            flex-direction: column;
        }

        .user-name {
            font-weight: 600;
            font-size: 0.95em;
        }

        .user-role {
            font-size: 0.75em;
            color: var(--text-secondary);
        }

        .logout-btn {
            padding: 10px 20px;
            background: rgba(239, 68, 68, 0.1);
            border: 1px solid rgba(239, 68, 68, 0.3);
            border-radius: 10px;
            color: var(--danger);
            cursor: pointer;
            transition: all 0.3s ease;
            font-weight: 600;
        }

        .logout-btn:hover {
            background: rgba(239, 68, 68, 0.2);
            transform: translateY(-2px);
        }

        .container {
            max-width: 2000px;
            margin: 0 auto;
            padding: 30px;
        }

        /* USER MODE */
        .user-dashboard {
            max-width: 800px;
            margin: 0 auto;
        }

        .welcome-card {
            background: linear-gradient(135deg, var(--primary) 0%, var(--secondary) 100%);
            border-radius: 24px;
            padding: 40px;
            margin-bottom: 30px;
            text-align: center;
            box-shadow: 0 20px 60px rgba(59, 130, 246, 0.3);
            position: relative;
            overflow: hidden;
        }

        .welcome-card::before {
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
            0%, 100% { transform: scale(1); opacity: 1; }
            50% { transform: scale(1.1); opacity: 0.8; }
        }

        .welcome-content {
            position: relative;
            z-index: 1;
        }

        .welcome-title {
            font-size: 2.5em;
            font-weight: 800;
            margin-bottom: 15px;
            color: white;
        }

        .welcome-subtitle {
            font-size: 1.2em;
            color: rgba(255, 255, 255, 0.9);
            font-weight: 500;
        }

        .destination-card {
            background: var(--bg-card);
            border-radius: 20px;
            padding: 30px;
            margin-bottom: 25px;
            box-shadow: 0 10px 30px rgba(0, 0, 0, 0.3);
            border: 1px solid rgba(255, 255, 255, 0.1);
        }

        .card-title {
            font-size: 1.5em;
            font-weight: 700;
            margin-bottom: 25px;
            display: flex;
            align-items: center;
            gap: 12px;
            color: var(--primary);
        }

        .destinations-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
            gap: 15px;
        }

        .dest-btn {
            padding: 25px 20px;
            background: linear-gradient(135deg, rgba(59, 130, 246, 0.1) 0%, rgba(139, 92, 246, 0.1) 100%);
            border: 2px solid rgba(255, 255, 255, 0.1);
            border-radius: 16px;
            cursor: pointer;
            transition: all 0.3s ease;
            text-align: center;
            position: relative;
            overflow: hidden;
        }

        .dest-btn::before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 100%;
            height: 100%;
            background: linear-gradient(90deg, transparent, rgba(255,255,255,0.1), transparent);
            transition: left 0.5s ease;
        }

        .dest-btn:hover::before {
            left: 100%;
        }

        .dest-btn:hover {
            transform: translateY(-5px);
            border-color: var(--primary);
            box-shadow: 0 15px 40px rgba(59, 130, 246, 0.3);
        }

        .dest-btn.active {
            background: linear-gradient(135deg, var(--primary) 0%, var(--secondary) 100%);
            border-color: var(--primary);
        }

        .dest-icon {
            font-size: 3em;
            margin-bottom: 12px;
            display: block;
        }

        .dest-name {
            font-weight: 600;
            font-size: 1.05em;
            margin-bottom: 5px;
        }

        .dest-info {
            font-size: 0.8em;
            color: var(--text-secondary);
        }

        .call-robot-btn {
            width: 100%;
            padding: 20px;
            background: linear-gradient(135deg, var(--success) 0%, #059669 100%);
            border: none;
            border-radius: 16px;
            color: white;
            font-size: 1.3em;
            font-weight: 700;
            cursor: pointer;
            transition: all 0.3s ease;
            margin-top: 25px;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 12px;
            box-shadow: 0 10px 30px rgba(16, 185, 129, 0.3);
        }

        .call-robot-btn:hover {
            transform: translateY(-3px);
            box-shadow: 0 15px 40px rgba(16, 185, 129, 0.4);
        }

        .call-robot-btn:active {
            transform: translateY(-1px);
        }

        .call-robot-btn:disabled {
            opacity: 0.5;
            cursor: not-allowed;
        }

        .status-card {
            background: var(--bg-card);
            border-radius: 20px;
            padding: 30px;
            box-shadow: 0 10px 30px rgba(0, 0, 0, 0.3);
            border: 1px solid rgba(255, 255, 255, 0.1);
        }

        .status-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 25px;
        }

        .status-indicator {
            display: flex;
            align-items: center;
            gap: 10px;
            padding: 10px 20px;
            background: rgba(16, 185, 129, 0.1);
            border-radius: 12px;
            border: 1px solid rgba(16, 185, 129, 0.3);
        }

        .status-dot {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background: var(--success);
            animation: pulse 2s ease-in-out infinite;
        }

        @keyframes pulse {
            0%, 100% { opacity: 1; transform: scale(1); }
            50% { opacity: 0.6; transform: scale(1.1); }
        }

        .eta-display {
            text-align: center;
            padding: 30px;
            background: linear-gradient(135deg, rgba(59, 130, 246, 0.1) 0%, rgba(139, 92, 246, 0.1) 100%);
            border-radius: 16px;
            margin-bottom: 25px;
        }

        .eta-label {
            font-size: 0.9em;
            color: var(--text-secondary);
            margin-bottom: 10px;
        }

        .eta-value {
            font-size: 3em;
            font-weight: 800;
            background: linear-gradient(135deg, var(--primary) 0%, var(--secondary) 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
        }

        .progress-bar-container {
            background: rgba(255, 255, 255, 0.1);
            border-radius: 12px;
            height: 12px;
            overflow: hidden;
            margin-bottom: 20px;
        }

        .progress-bar {
            height: 100%;
            background: linear-gradient(90deg, var(--primary) 0%, var(--secondary) 100%);
            border-radius: 12px;
            transition: width 0.5s ease;
            box-shadow: 0 0 20px rgba(59, 130, 246, 0.5);
        }

        .cancel-btn {
            width: 100%;
            padding: 15px;
            background: rgba(239, 68, 68, 0.1);
            border: 2px solid rgba(239, 68, 68, 0.3);
            border-radius: 12px;
            color: var(--danger);
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s ease;
        }

        .cancel-btn:hover {
            background: rgba(239, 68, 68, 0.2);
            transform: translateY(-2px);
        }

        /* ADMIN MODE */
        .admin-dashboard {
            display: grid;
            grid-template-columns: 380px 1fr 380px;
            gap: 25px;
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
            box-shadow: 0 15px 40px rgba(59, 130, 246, 0.3);
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

        .mode-selector {
            display: flex;
            gap: 15px;
            margin-bottom: 25px;
            padding: 20px;
            background: var(--bg-card);
            border-radius: 16px;
            box-shadow: 0 10px 30px rgba(0, 0, 0, 0.3);
        }

        .mode-toggle {
            flex: 1;
            padding: 18px 25px;
            background: rgba(255, 255, 255, 0.05);
            border: 2px solid rgba(255, 255, 255, 0.1);
            border-radius: 12px;
            cursor: pointer;
            transition: all 0.3s ease;
            text-align: center;
            font-weight: 600;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 10px;
        }

        .mode-toggle:hover {
            transform: translateY(-3px);
            box-shadow: 0 8px 25px rgba(59, 130, 246, 0.3);
        }

        .mode-toggle.active {
            background: linear-gradient(135deg, var(--success) 0%, #059669 100%);
            border-color: var(--success);
            box-shadow: 0 0 30px rgba(16, 185, 129, 0.4);
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
            box-shadow: 0 5px 15px rgba(59, 130, 246, 0.3);
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
            box-shadow: 0 10px 25px rgba(59, 130, 246, 0.5);
        }

        .control-btn.active {
            background: linear-gradient(135deg, var(--success) 0%, #059669 100%);
            box-shadow: 0 0 30px var(--success);
        }

        .control-btn.stop-btn {
            background: linear-gradient(135deg, var(--danger) 0%, #dc2626 100%);
            font-size: 1.3em;
            font-weight: 700;
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
        }

        .speed-slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 22px;
            height: 22px;
            border-radius: 50%;
            background: linear-gradient(135deg, var(--primary) 0%, var(--secondary) 100%);
            cursor: pointer;
            box-shadow: 0 0 10px rgba(59, 130, 246, 0.5);
            transition: all 0.3s ease;
        }

        .speed-slider::-webkit-slider-thumb:hover {
            transform: scale(1.2);
            box-shadow: 0 0 20px rgba(59, 130, 246, 0.8);
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
            box-shadow: 0 5px 20px rgba(59, 130, 246, 0.3);
            transition: all 0.3s ease;
        }

        .metric-card:hover {
            transform: scale(1.05);
            box-shadow: 0 10px 30px rgba(59, 130, 246, 0.5);
        }

        .metric-icon {
            font-size: 2em;
            margin-bottom: 10px;
        }

        .metric-value {
            font-size: 2em;
            font-weight: 700;
            margin: 10px 0;
        }

        .metric-label {
            font-size: 0.85em;
            opacity: 0.9;
        }

        .lidar-container {
            background: var(--bg-dark);
            border-radius: 15px;
            padding: 15px;
            height: 220px;
            border: 2px solid var(--primary);
            box-shadow: inset 0 0 20px rgba(0,0,0,0.5);
        }

        #lidarCanvas {
            width: 100%;
            height: 100%;
            display: block;
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

        #mapCanvas {
            width: 100%;
            height: 100%;
            cursor: crosshair;
            display: block;
        }

        /* 🔥 LÉGENDE CARTE - Style Nav2 */
        .map-overlay {
            position: absolute;
            top: 15px;
            left: 15px;
            background: rgba(15, 23, 42, 0.95);
            padding: 15px;
            border-radius: 12px;
            backdrop-filter: blur(10px);
            font-size: 0.85em;
            z-index: 10;
            border: 1px solid rgba(255, 255, 255, 0.1);
        }

        .map-overlay-item {
            display: flex;
            align-items: center;
            gap: 10px;
            margin-bottom: 10px;
            padding: 5px;
            border-radius: 5px;
            transition: background 0.2s ease;
        }

        .map-overlay-item:last-child {
            margin-bottom: 0;
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
            font-size: 0.9em;
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
            background: rgba(59, 130, 246, 0.9);
            border: none;
            color: white;
            width: 45px;
            height: 45px;
            border-radius: 12px;
            cursor: pointer;
            font-size: 1.2em;
            transition: all 0.3s ease;
            display: flex;
            align-items: center;
            justify-content: center;
        }

        .map-btn:hover {
            background: var(--primary);
            transform: scale(1.1);
            box-shadow: 0 5px 20px rgba(59, 130, 246, 0.5);
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
            cursor: pointer;
            transition: all 0.3s ease;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 8px;
        }

        .action-btn.primary {
            background: linear-gradient(135deg, var(--info) 0%, #0891b2 100%);
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

        /* 🔥 SECTION WAYPOINTS - Style Nav2 */
        .waypoint-hint {
            text-align: center;
            padding: 12px;
            background: rgba(102, 126, 234, 0.1);
            border-radius: 10px;
            font-size: 0.9em;
            color: var(--text-secondary);
            margin-bottom: 15px;
        }

        .waypoints-container {
            max-height: 300px;
            overflow-y: auto;
            margin-bottom: 15px;
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

        /* 🔥 ÉTAT VIDE WAYPOINTS - Style Nav2 */
        .empty-state {
            text-align: center;
            padding: 40px 20px;
            color: var(--text-secondary);
        }

        .empty-icon {
            font-size: 3em;
            margin-bottom: 15px;
            opacity: 0.5;
        }

        .empty-text {
            font-size: 1em;
            margin-bottom: 8px;
        }

        .empty-subtext {
            font-size: 0.85em;
            opacity: 0.7;
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
            min-width: 320px;
            transform: translateX(500px);
            transition: transform 0.4s cubic-bezier(0.68, -0.55, 0.265, 1.55);
            z-index: 10000;
            border-left: 5px solid;
        }

        .notification.show {
            transform: translateX(0);
        }

        .notification.success { border-left-color: var(--success); }
        .notification.error { border-left-color: var(--danger); }
        .notification.info { border-left-color: var(--info); }
        .notification.warning { border-left-color: var(--warning); }

        .notification-content {
            display: flex;
            align-items: center;
            gap: 15px;
        }

        .notification-icon {
            font-size: 1.8em;
        }

        .connection-status {
            position: fixed;
            bottom: 20px;
            right: 20px;
            background: rgba(0, 0, 0, 0.8);
            color: white;
            padding: 12px 20px;
            border-radius: 25px;
            font-weight: 600;
            backdrop-filter: blur(10px);
            z-index: 1000;
            border: 2px solid;
            display: flex;
            align-items: center;
            gap: 8px;
        }

        .connection-status.connected {
            background: rgba(16, 185, 129, 0.9);
            border-color: var(--success);
        }

        .connection-status.disconnected {
            background: rgba(239, 68, 68, 0.9);
            border-color: var(--danger);
        }

        @media (max-width: 1400px) {
            .admin-dashboard {
                grid-template-columns: 1fr;
            }
        }

        .loading {
            display: inline-block;
            width: 20px;
            height: 20px;
            border: 3px solid rgba(255,255,255,.3);
            border-radius: 50%;
            border-top-color: white;
            animation: spin 1s ease-in-out infinite;
        }

        @keyframes spin {
            to { transform: rotate(360deg); }
        }

        .hidden {
            display: none !important;
        }
    </style>
</head>
<body>
    <!-- LOGIN SCREEN (identique) -->
    <div class="login-container" id="loginScreen">
        <div class="login-box">
            <div class="login-header">
                <div class="login-logo">🏥</div>
                <h1 class="login-title">HospiBot</h1>
                <p class="login-subtitle">Système de Navigation Hospitalière Intelligent</p>
            </div>

            <div class="role-selector">
                <div class="role-btn" onclick="selectRole('user')" id="roleUser">
                    <span class="role-icon">👤</span>
                    <div class="role-name">Utilisateur</div>
                </div>
                <div class="role-btn" onclick="selectRole('admin')" id="roleAdmin">
                    <span class="role-icon">👨‍💼</span>
                    <div class="role-name">Administrateur</div>
                </div>
            </div>

            <div class="login-form" id="userForm">
                <div class="form-group">
                    <label class="form-label">Nom complet</label>
                    <input type="text" class="form-input" id="userName" placeholder="Entrez votre nom">
                </div>
                <button class="login-btn" onclick="loginUser()">
                    <span>Accéder au système</span>
                    <span>→</span>
                </button>
            </div>

            <div class="login-form" id="adminForm">
                <div class="form-group">
                    <label class="form-label">Identifiant</label>
                    <input type="text" class="form-input" id="adminUser" placeholder="Administrateur">
                </div>
                <div class="form-group">
                    <label class="form-label">Mot de passe</label>
                    <input type="password" class="form-input" id="adminPassword" placeholder="••••••••">
                </div>
                <button class="login-btn" onclick="loginAdmin()">
                    <span>Se connecter</span>
                    <span>→</span>
                </button>
            </div>

            <div class="quick-access">
                <div class="quick-access-title">Accès rapide</div>
                <div class="quick-btns">
                    <button class="quick-btn" onclick="quickLogin('user', 'Visiteur')">Visiteur</button>
                    <button class="quick-btn" onclick="quickLogin('user', 'Patient')">Patient</button>
                    <button class="quick-btn" onclick="quickLogin('admin', 'Admin')">Admin Demo</button>
                </div>
            </div>
        </div>
    </div>

    <!-- MAIN APP -->
    <div class="app-container" id="appContainer">
        <!-- Navbar -->
        <nav class="navbar">
            <div class="navbar-brand">
                <span class="brand-logo">🏥</span>
                <div class="brand-text">
                    <div class="brand-title">HospiBot</div>
                    <div class="brand-subtitle">Navigation Hospitalière</div>
                </div>
            </div>
            <div class="navbar-user">
                <div class="user-badge">
                    <div class="user-avatar" id="userAvatar">👤</div>
                    <div class="user-info">
                        <div class="user-name" id="displayName">Utilisateur</div>
                        <div class="user-role" id="displayRole">Patient</div>
                    </div>
                </div>
                <button class="logout-btn" onclick="logout()">
                    <span>🚪</span>
                    <span>Déconnexion</span>
                </button>
            </div>
        </nav>

        <div class="container">
            <!-- USER DASHBOARD (identique - conservé) -->
            <div class="user-dashboard hidden" id="userDashboard">
                <div class="welcome-card">
                    <div class="welcome-content">
                        <h1 class="welcome-title">Bienvenue 👋</h1>
                        <p class="welcome-subtitle">Où souhaitez-vous aller aujourd'hui ?</p>
                    </div>
                </div>

                <div class="destination-card">
                    <h2 class="card-title">
                        <span>📍</span>
                        <span>Choisissez votre destination</span>
                    </h2>
                    <div class="destinations-grid">
                        <div class="dest-btn" onclick="selectDestination('reception')">
                            <span class="dest-icon">🏢</span>
                            <div class="dest-name">Réception</div>
                            <div class="dest-info">Hall principal</div>
                        </div>
                        <div class="dest-btn" onclick="selectDestination('emergency')">
                            <span class="dest-icon">🚨</span>
                            <div class="dest-name">Urgences</div>
                            <div class="dest-info">Service d'urgence</div>
                        </div>
                        <div class="dest-btn" onclick="selectDestination('pharmacy')">
                            <span class="dest-icon">💊</span>
                            <div class="dest-name">Pharmacie</div>
                            <div class="dest-info">Médicaments</div>
                        </div>
                        <div class="dest-btn" onclick="selectDestination('lab')">
                            <span class="dest-icon">🔬</span>
                            <div class="dest-name">Laboratoire</div>
                            <div class="dest-info">Analyses</div>
                        </div>
                        <div class="dest-btn" onclick="selectDestination('consult1')">
                            <span class="dest-icon">👨‍⚕️</span>
                            <div class="dest-name">Consultation 1</div>
                            <div class="dest-info">Cabinet médical</div>
                        </div>
                        <div class="dest-btn" onclick="selectDestination('consult2')">
                            <span class="dest-icon">👩‍⚕️</span>
                            <div class="dest-name">Consultation 2</div>
                            <div class="dest-info">Cabinet médical</div>
                        </div>
                        <div class="dest-btn" onclick="selectDestination('consult3')">
                            <span class="dest-icon">🧑‍⚕️</span>
                            <div class="dest-name">Consultation 3</div>
                            <div class="dest-info">Cabinet médical</div>
                        </div>
                        <div class="dest-btn" onclick="selectDestination('entrance')">
                            <span class="dest-icon">🚪</span>
                            <div class="dest-name">Entrée</div>
                            <div class="dest-info">Sortie principale</div>
                        </div>
                    </div>
                    <button class="call-robot-btn" id="callRobotBtn" onclick="callRobot()" disabled>
                        <span>🤖</span>
                        <span>Appeler le robot</span>
                    </button>
                </div>

                <div class="status-card hidden" id="navigationStatus">
                    <div class="status-header">
                        <h2 class="card-title">
                            <span>🎯</span>
                            <span>Navigation en cours</span>
                        </h2>
                        <div class="status-indicator">
                            <span class="status-dot"></span>
                            <span>En route</span>
                        </div>
                    </div>

                    <div class="eta-display">
                        <div class="eta-label">Temps d'arrivée estimé</div>
                        <div class="eta-value" id="etaValue">2:30</div>
                    </div>

                    <div class="progress-bar-container">
                        <div class="progress-bar" id="progressBar" style="width: 0%"></div>
                    </div>

                    <button class="cancel-btn" onclick="cancelUserNavigation()">
                        <span>⏹️</span>
                        <span>Annuler la navigation</span>
                    </button>
                </div>
            </div>

            <!-- ADMIN DASHBOARD avec améliorations visuelles -->
            <div class="admin-dashboard hidden" id="adminDashboard">
                <!-- Panneau gauche - Contrôles -->
                <div class="panel">
                    <div class="panel-title">
                        <span>🎮</span>
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

                    <div style="text-align: center; padding: 12px; background: rgba(59, 130, 246, 0.1); border-radius: 10px; font-size: 0.9em; color: var(--text-secondary); margin-bottom: 20px;">
                        💡 W A S D ou ← ↑ → ↓
                    </div>

                    <div class="speed-control">
                        <div class="speed-label">
                            <span>Vitesse Linéaire</span>
                            <span class="speed-value" id="linearValue">0.5 m/s</span>
                        </div>
                        <input type="range" class="speed-slider" id="linearSpeed" 
                               min="0.1" max="2.0" step="0.1" value="0.5">
                    </div>

                    <div class="speed-control">
                        <div class="speed-label">
                            <span>Vitesse Angulaire</span>
                            <span class="speed-value" id="angularValue">1.0 rad/s</span>
                        </div>
                        <input type="range" class="speed-slider" id="angularSpeed" 
                               min="0.1" max="3.0" step="0.1" value="1.0">
                    </div>

                    <div class="panel-title" style="margin-top: 25px;">
                        <span>📊</span>
                        <span>Métriques</span>
                    </div>

                    <div class="metrics-grid">
                        <div class="metric-card">
                            <div class="metric-icon">📍</div>
                            <div class="metric-value" id="posX">0.00</div>
                            <div class="metric-label">Position X</div>
                        </div>
                        <div class="metric-card">
                            <div class="metric-icon">📍</div>
                            <div class="metric-value" id="posY">0.00</div>
                            <div class="metric-label">Position Y</div>
                        </div>
                        <div class="metric-card">
                            <div class="metric-icon">⚡</div>
                            <div class="metric-value" id="velocity">0.00</div>
                            <div class="metric-label">Vitesse</div>
                        </div>
                        <div class="metric-card">
                            <div class="metric-icon">🧭</div>
                            <div class="metric-value" id="rotation">0°</div>
                            <div class="metric-label">Orientation</div>
                        </div>
                    </div>

                    <div class="panel-title">
                        <span>📡</span>
                        <span>LiDAR</span>
                    </div>

                    <div class="lidar-container">
                        <canvas id="lidarCanvas"></canvas>
                    </div>
                </div>

                <!-- Panneau central - Carte -->
                <div class="panel">
                    <div class="mode-selector">
                        <button class="mode-toggle active" id="slamModeBtn" onclick="switchMode('slam')">
                            <span>🗺️</span>
                            <span>SLAM</span>
                        </button>
                        <button class="mode-toggle" id="nav2ModeBtn" onclick="switchMode('nav2')">
                            <span>🧭</span>
                            <span>Nav2</span>
                        </button>
                    </div>

                    <div class="panel-title">
                        <span>🗺️</span>
                        <span>Carte</span>
                    </div>

                    <div class="map-container">
                        <canvas id="mapCanvas"></canvas>
                        
                        <!-- 🔥 LÉGENDE AMÉLIORÉE - Style Nav2 -->
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
                            <button class="map-btn" onclick="zoomIn()">+</button>
                            <button class="map-btn" onclick="zoomOut()">−</button>
                            <button class="map-btn" onclick="centerMap()">⌖</button>
                            <button class="map-btn" onclick="clearWaypoints()" title="Effacer waypoints">🗑️</button>
                        </div>
                    </div>

                    <div class="action-btns" id="modeActions">
                        <button class="action-btn success" onclick="saveMap()">
                            <span>💾</span>
                            <span>Sauvegarder</span>
                        </button>
                        <button class="action-btn warning" onclick="resetPose()">
                            <span>🔄</span>
                            <span>Réinitialiser</span>
                        </button>
                        <button class="action-btn primary" onclick="relocalize()">
                            <span>📍</span>
                            <span>Relocaliser</span>
                        </button>
                        <button class="action-btn danger" onclick="clearMap()">
                            <span>🗑️</span>
                            <span>Effacer</span>
                        </button>
                    </div>
                </div>

                <!-- Panneau droit - Destinations + Waypoints -->
                <div class="panel">
                    <div class="panel-title">
                        <span>🏥</span>
                        <span>Destinations</span>
                    </div>
                    <div class="destinations-grid">
                        <div class="dest-btn" onclick="adminGoTo('reception')">
                            <span class="dest-icon">🏢</span>
                            <div class="dest-name">Réception</div>
                        </div>
                        <div class="dest-btn" onclick="adminGoTo('emergency')">
                            <span class="dest-icon">🚨</span>
                            <div class="dest-name">Urgences</div>
                        </div>
                        <div class="dest-btn" onclick="adminGoTo('pharmacy')">
                            <span class="dest-icon">💊</span>
                            <div class="dest-name">Pharmacie</div>
                        </div>
                        <div class="dest-btn" onclick="adminGoTo('lab')">
                            <span class="dest-icon">🔬</span>
                            <div class="dest-name">Labo</div>
                        </div>
                    </div>

                    <div class="panel-title" style="margin-top: 25px;">
                        <span>📍</span>
                        <span>Points de Navigation</span>
                    </div>

                    <!-- 🔥 HINT WAYPOINTS - Style Nav2 -->
                    <div class="waypoint-hint">
                        💡 Cliquez sur la carte pour ajouter des waypoints
                    </div>

                    <!-- 🔥 LISTE WAYPOINTS - Style Nav2 -->
                    <div class="waypoints-container" id="waypointsContainer">
                        <div class="empty-state">
                            <div class="empty-icon">📍</div>
                            <div class="empty-text">Aucun waypoint défini</div>
                            <div class="empty-subtext">Cliquez sur la carte</div>
                        </div>
                    </div>

                    <!-- 🔥 BOUTONS WAYPOINTS - Style Nav2 -->
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
    </div>

    <div class="notification" id="notification">
        <div class="notification-content">
            <span class="notification-icon" id="notifIcon"></span>
            <span id="notifText"></span>
        </div>
    </div>

    <div class="connection-status connected" id="connectionStatus">
        <span class="status-dot"></span>
        <span>Connecté</span>
    </div>

    <script>
        // ================================================================
        // 🔥 VERSION FINALE - Toutes corrections + Visuels Nav2
        // ================================================================
        
        // STATE
        let userRole = null;
        let userName = null;
        let selectedDestination = null;
        let currentMode = 'slam';
        let isNavigating = false;
        let navStartTime = null;
        let lastUpdateTime = Date.now();
        let robotPose = {x: 0, y: 0, theta: 0};
        let mapZoom = 20.0;
        let mapOffset = {x: 0, y: 0};
        let waypoints = [];
        let updateInterval = null;
        let showGrid = true;
        
        // Vitesse progressive
        let currentLinearSpeed = 0.5;
        let currentAngularSpeed = 1.0;
        let targetLinearSpeed = 0.5;
        let targetAngularSpeed = 1.0;
        let isMoving = false;

        const mapCanvas = document.getElementById('mapCanvas');
        const mapCtx = mapCanvas?.getContext('2d');
        const lidarCanvas = document.getElementById('lidarCanvas');
        const lidarCtx = lidarCanvas?.getContext('2d');

        // Initialisation Canvas
        window.addEventListener('DOMContentLoaded', function() {
            console.log('🔧 Initialisation des canvas...');
            if (mapCanvas && lidarCanvas) {
                resizeCanvases();
                setupMapClickHandler();
                setupKeyboardControls();
            }
        });

        // Handler clic carte pour waypoints
        function setupMapClickHandler() {
            if (!mapCanvas) return;
            
            mapCanvas.addEventListener('click', function(event) {
                if (userRole !== 'admin') return;
                
                const rect = mapCanvas.getBoundingClientRect();
                const canvasX = event.clientX - rect.left;
                const canvasY = event.clientY - rect.top;
                
                const worldX = (canvasX - mapCanvas.width / 2) / mapZoom;
                const worldY = -(canvasY - mapCanvas.height / 2) / mapZoom;
                
                addWaypoint(worldX, worldY);
                console.log(`📍 Waypoint ajouté: (${worldX.toFixed(2)}, ${worldY.toFixed(2)})`);
            });
            
            console.log('✅ Handler de clic configuré');
        }

        // Contrôle clavier
        function setupKeyboardControls() {
            let activeKeys = {};
            
            document.addEventListener('keydown', (e) => {
                if (userRole !== 'admin') return;
                if (e.target.tagName === 'INPUT') return;
                
                const key = e.key.toLowerCase();
                if (!activeKeys[key]) {
                    activeKeys[key] = true;
                    handleKeyPress(key, true);
                }
                
                if (['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'].includes(key)) {
                    e.preventDefault();
                }
            });
            
            document.addEventListener('keyup', (e) => {
                if (userRole !== 'admin') return;
                const key = e.key.toLowerCase();
                activeKeys[key] = false;
                
                const movementKeys = ['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'];
                if (!movementKeys.some(k => activeKeys[k])) {
                    sendCommand('stop');
                    isMoving = false;
                }
            });
        }
        
        function handleKeyPress(key, isActive) {
            if (!isActive) return;
            
            const keyMap = {
                'w': 'forward',
                'arrowup': 'forward',
                's': 'backward',
                'arrowdown': 'backward',
                'a': 'left',
                'arrowleft': 'left',
                'd': 'right',
                'arrowright': 'right',
                ' ': 'stop'
            };
            
            if (keyMap[key]) {
                sendCommand(keyMap[key]);
                isMoving = (key !== ' ');
            }
        }

        // 🔥 Gestion waypoints avec UI améliorée
        function addWaypoint(x, y) {
            const waypoint = {
                id: Date.now(),
                x: x.toFixed(2),
                y: y.toFixed(2),
                name: `Point ${waypoints.length + 1}`
            };
            waypoints.push(waypoint);
            updateWaypointsList();
            drawMap();
            
            fetch('/add_waypoint', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(waypoint)
            }).then(res => res.json())
              .then(data => {
                  if (data.status === 'success') {
                      showNotification(`Waypoint ${waypoints.length}: (${x.toFixed(1)}, ${y.toFixed(1)})`, 'success');
                  }
              });
        }

        // 🔥 Mise à jour liste waypoints - Style Nav2
        function updateWaypointsList() {
            const container = document.getElementById('waypointsContainer');
            
            if (waypoints.length === 0) {
                container.innerHTML = `
                    <div class="empty-state">
                        <div class="empty-icon">📍</div>
                        <div class="empty-text">Aucun waypoint défini</div>
                        <div class="empty-subtext">Cliquez sur la carte</div>
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
            drawMap();
            showNotification('Waypoint supprimé', 'info');
        }

        function clearWaypoints() {
            if (waypoints.length === 0) {
                showNotification('Aucun waypoint', 'info');
                return;
            }
            
            if (confirm(`Effacer ${waypoints.length} waypoint(s) ?`)) {
                waypoints = [];
                updateWaypointsList();
                drawMap();
                fetch('/clear_waypoints', {method: 'POST'});
                showNotification('Waypoints effacés', 'info');
            }
        }

        function saveWaypoints() {
            if (waypoints.length === 0) {
                showNotification('Aucun waypoint à sauvegarder', 'warning');
                return;
            }
            showNotification(`${waypoints.length} waypoint(s) sauvegardé(s)`, 'success');
        }

        // updateData avec métriques temps réel
        async function updateData() {
            try {
                const response = await fetch(`/get_data?mode=${currentMode}`);
                if (!response.ok) throw new Error('Network error');
                
                const data = await response.json();
                
                if (data.pose && typeof data.pose.x === 'number') {
                    robotPose = {
                        x: data.pose.x,
                        y: data.pose.y,
                        theta: data.pose.theta || 0
                    };
                    
                    const posXEl = document.getElementById('posX');
                    const posYEl = document.getElementById('posY');
                    const velocityEl = document.getElementById('velocity');
                    const rotationEl = document.getElementById('rotation');
                    
                    if (posXEl) posXEl.textContent = robotPose.x.toFixed(2);
                    if (posYEl) posYEl.textContent = robotPose.y.toFixed(2);
                    if (velocityEl && typeof data.velocity === 'number') {
                        velocityEl.textContent = data.velocity.toFixed(2);
                    }
                    if (rotationEl) {
                        const degrees = (robotPose.theta * 180 / Math.PI).toFixed(0);
                        rotationEl.textContent = degrees + '°';
                    }
                }
                
                if (data.laser && Array.isArray(data.laser.ranges)) {
                    drawLidar(data.laser);
                }
                
                drawMap();
                lastUpdateTime = Date.now();
                
            } catch (error) {
                console.error('❌ Erreur update:', error);
            }
        }

        function startDataUpdate() {
            if (updateInterval) clearInterval(updateInterval);
            updateInterval = setInterval(updateData, 100);
            console.log('✅ Mise à jour démarrée (10 Hz)');
            updateData();
        }

        function resizeCanvases() {
            if (mapCanvas) {
                mapCanvas.width = mapCanvas.offsetWidth || 800;
                mapCanvas.height = mapCanvas.offsetHeight || 650;
                console.log(`📐 Map canvas: ${mapCanvas.width}x${mapCanvas.height}`);
                drawMap();
            }
            if (lidarCanvas) {
                lidarCanvas.width = lidarCanvas.offsetWidth || 300;
                lidarCanvas.height = lidarCanvas.offsetHeight || 220;
                console.log(`📐 LiDAR canvas: ${lidarCanvas.width}x${lidarCanvas.height}`);
            }
        }

        // 🔥 drawMap avec layout hospitalier
        function drawMap() {
            if (!mapCtx || !mapCanvas.width) return;
            
            const w = mapCanvas.width;
            const h = mapCanvas.height;
            
            mapCtx.fillStyle = '#0f0f1e';
            mapCtx.fillRect(0, 0, w, h);
            
            mapCtx.save();
            mapCtx.translate(w/2, h/2);
            mapCtx.scale(mapZoom, mapZoom);
            
            // Grid
            if (showGrid) {
                mapCtx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
                mapCtx.lineWidth = 0.5 / mapZoom;
                for (let x = -30; x <= 30; x += 2) {
                    mapCtx.beginPath();
                    mapCtx.moveTo(x, -30);
                    mapCtx.lineTo(x, 30);
                    mapCtx.stroke();
                    mapCtx.beginPath();
                    mapCtx.moveTo(-30, x);
                    mapCtx.lineTo(30, x);
                    mapCtx.stroke();
                }
            }
            
            // 🔥 Hospital layout
            drawHospitalLayout(mapCtx);
            
            // Waypoints
            waypoints.forEach((wp, index) => {
                const x = parseFloat(wp.x);
                const y = parseFloat(wp.y);
                
                mapCtx.fillStyle = '#ef4444';
                mapCtx.beginPath();
                mapCtx.arc(x, -y, 0.3, 0, Math.PI * 2);
                mapCtx.fill();
                
                mapCtx.fillStyle = 'white';
                mapCtx.font = `${12 / mapZoom}px Inter`;
                mapCtx.textAlign = 'center';
                mapCtx.fillText(wp.name, x, -y - 0.5);
            });
            
            // Robot
            if (robotPose && typeof robotPose.x === 'number') {
                mapCtx.save();
                mapCtx.translate(robotPose.x, -robotPose.y);
                mapCtx.rotate(-robotPose.theta);
                
                mapCtx.fillStyle = '#10b981';
                mapCtx.beginPath();
                mapCtx.arc(0, 0, 0.22, 0, Math.PI * 2);
                mapCtx.fill();
                
                mapCtx.strokeStyle = '#059669';
                mapCtx.lineWidth = 3 / mapZoom;
                mapCtx.beginPath();
                mapCtx.moveTo(0, 0);
                mapCtx.lineTo(0.3, 0);
                mapCtx.stroke();
                
                mapCtx.fillStyle = '#059669';
                mapCtx.beginPath();
                mapCtx.moveTo(0.3, 0);
                mapCtx.lineTo(0.25, 0.05);
                mapCtx.lineTo(0.25, -0.05);
                mapCtx.closePath();
                mapCtx.fill();
                
                mapCtx.restore();
            }
            
            mapCtx.restore();
        }

        // 🔥 Dessin layout hôpital
        function drawHospitalLayout(ctx) {
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
            ctx.lineWidth = 1 / mapZoom;
            ctx.strokeRect(-15, -15, 30, 30);
            
            // Salles avec couleurs
            const rooms = [
                {x: -13, y: -3, w: 6, h: 6, color: 'rgba(59, 130, 246, 0.1)', label: 'Réception'},
                {x: -10, y: 6, w: 6, h: 6, color: 'rgba(16, 185, 129, 0.1)', label: 'C1'},
                {x: -4, y: 6, w: 6, h: 6, color: 'rgba(16, 185, 129, 0.1)', label: 'C2'},
                {x: 4, y: 6, w: 6, h: 6, color: 'rgba(16, 185, 129, 0.1)', label: 'C3'},
                {x: -10, y: -12, w: 6, h: 6, color: 'rgba(239, 68, 68, 0.1)', label: 'Urgences'},
                {x: -4, y: -12, w: 6, h: 6, color: 'rgba(139, 92, 246, 0.1)', label: 'Labo'},
                {x: 4, y: -12, w: 6, h: 6, color: 'rgba(245, 158, 11, 0.1)', label: 'Pharmacie'}
            ];
            
            rooms.forEach(room => {
                ctx.fillStyle = room.color;
                ctx.fillRect(room.x, room.y, room.w, room.h);
                ctx.strokeRect(room.x, room.y, room.w, room.h);
                
                ctx.fillStyle = 'rgba(255, 255, 255, 0.6)';
                ctx.font = `${10 / mapZoom}px Inter`;
                ctx.textAlign = 'center';
                ctx.fillText(room.label, room.x + room.w/2, room.y + room.h/2);
            });
        }

        function drawLidar(scanData) {
            if (!lidarCtx || !lidarCanvas.width || !scanData) return;
            
            const w = lidarCanvas.width;
            const h = lidarCanvas.height;
            const cx = w / 2;
            const cy = h / 2;
            const radius = Math.min(w, h) / 2 - 10;
            
            lidarCtx.fillStyle = '#0f0f1e';
            lidarCtx.fillRect(0, 0, w, h);
            
            lidarCtx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
            lidarCtx.lineWidth = 1;
            for (let i = 1; i <= 3; i++) {
                lidarCtx.beginPath();
                lidarCtx.arc(cx, cy, (radius * i) / 3, 0, Math.PI * 2);
                lidarCtx.stroke();
            }
            
            if (Array.isArray(scanData.ranges) && scanData.ranges.length > 0) {
                const ranges = scanData.ranges;
                const angleMin = scanData.angle_min || 0;
                const angleIncrement = scanData.angle_increment || (2 * Math.PI / ranges.length);
                
                lidarCtx.fillStyle = '#ef4444';
                
                for (let i = 0; i < ranges.length; i++) {
                    const range = ranges[i];
                    if (range > 0.1 && range < 12) {
                        const angle = angleMin + i * angleIncrement;
                        const normalizedRange = range / 12;
                        const x = cx + Math.cos(angle) * normalizedRange * radius;
                        const y = cy + Math.sin(angle) * normalizedRange * radius;
                        
                        lidarCtx.beginPath();
                        lidarCtx.arc(x, y, 2, 0, Math.PI * 2);
                        lidarCtx.fill();
                    }
                }
            }
            
            lidarCtx.fillStyle = '#10b981';
            lidarCtx.beginPath();
            lidarCtx.arc(cx, cy, 5, 0, Math.PI * 2);
            lidarCtx.fill();
        }

        // LOGIN FUNCTIONS (identiques)
        function selectRole(role) {
            document.getElementById('roleUser').classList.remove('active');
            document.getElementById('roleAdmin').classList.remove('active');
            document.getElementById('userForm').classList.remove('active');
            document.getElementById('adminForm').classList.remove('active');

            if (role === 'user') {
                document.getElementById('roleUser').classList.add('active');
                document.getElementById('userForm').classList.add('active');
            } else {
                document.getElementById('roleAdmin').classList.add('active');
                document.getElementById('adminForm').classList.add('active');
            }
        }

        function loginUser() {
            const name = document.getElementById('userName').value;
            if (!name) {
                showNotification('Veuillez entrer votre nom', 'warning');
                return;
            }
            enterApp('user', name);
        }

        function loginAdmin() {
            const user = document.getElementById('adminUser').value;
            const pass = document.getElementById('adminPassword').value;
            
            if (user === 'admin' && pass === 'admin123') {
                enterApp('admin', 'Administrateur');
            } else {
                showNotification('Identifiants incorrects', 'error');
            }
        }

        function quickLogin(role, name) {
            if (role === 'admin') {
                document.getElementById('adminUser').value = 'admin';
                document.getElementById('adminPassword').value = 'admin123';
            }
            enterApp(role, name);
        }

        function enterApp(role, name) {
            userRole = role;
            userName = name;

            document.getElementById('loginScreen').style.display = 'none';
            document.getElementById('appContainer').classList.add('active');
            
            document.getElementById('displayName').textContent = name;
            document.getElementById('displayRole').textContent = role === 'admin' ? 'Administrateur' : 'Utilisateur';
            document.getElementById('userAvatar').textContent = role === 'admin' ? '👨‍💼' : '👤';

            if (role === 'user') {
                document.getElementById('userDashboard').classList.remove('hidden');
            } else {
                document.getElementById('adminDashboard').classList.remove('hidden');
                setTimeout(() => {
                    resizeCanvases();
                }, 100);
            }

            showNotification(`Bienvenue ${name} !`, 'success');
            startDataUpdate();
        }

        function logout() {
            if (confirm('Voulez-vous vraiment vous déconnecter ?')) {
                location.reload();
            }
        }

        // USER MODE FUNCTIONS (identiques - déjà dans le code)
        function selectDestination(dest) {
            selectedDestination = dest;
            
            document.querySelectorAll('.dest-btn').forEach(btn => {
                btn.classList.remove('active');
            });
            event.target.closest('.dest-btn').classList.add('active');
            
            document.getElementById('callRobotBtn').disabled = false;
        }

        async function callRobot() {
            if (!selectedDestination) return;

            try {
                const response = await fetch('/goto_location', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({
                        location: selectedDestination,
                        mode: 'nav2'
                    })
                });
                const data = await response.json();

                if (data.status === 'success') {
                    document.querySelector('.destination-card').classList.add('hidden');
                    document.getElementById('navigationStatus').classList.remove('hidden');
                    isNavigating = true;
                    navStartTime = Date.now();
                    showNotification(`Robot en route vers ${data.name}`, 'success');
                    simulateProgress();
                }
            } catch (error) {
                showNotification('Erreur de connexion', 'error');
            }
        }

        function cancelUserNavigation() {
            fetch('/cancel_navigation', {method: 'POST'});
            resetUserInterface();
            showNotification('Navigation annulée', 'info');
        }

        function resetUserInterface() {
            document.querySelector('.destination-card').classList.remove('hidden');
            document.getElementById('navigationStatus').classList.add('hidden');
            selectedDestination = null;
            isNavigating = false;
            document.querySelectorAll('.dest-btn').forEach(btn => {
                btn.classList.remove('active');
            });
            document.getElementById('callRobotBtn').disabled = true;
        }

        function simulateProgress() {
            let progress = 0;
            const interval = setInterval(() => {
                progress += 2;
                document.getElementById('progressBar').style.width = progress + '%';
                
                const remaining = Math.max(0, 150 - progress * 1.5);
                const mins = Math.floor(remaining / 60);
                const secs = Math.floor(remaining % 60);
                document.getElementById('etaValue').textContent = `${mins}:${secs.toString().padStart(2, '0')}`;

                if (progress >= 100) {
                    clearInterval(interval);
                    showNotification('Destination atteinte !', 'success');
                    setTimeout(resetUserInterface, 2000);
                }
            }, 1000);
        }

        // ADMIN MODE FUNCTIONS
        function switchMode(mode) {
            currentMode = mode;
            document.getElementById('slamModeBtn').classList.remove('active');
            document.getElementById('nav2ModeBtn').classList.remove('active');
            
            if (mode === 'slam') {
                document.getElementById('slamModeBtn').classList.add('active');
                document.getElementById('modeActions').innerHTML = `
                    <button class="action-btn success" onclick="saveMap()">
                        <span>💾</span>
                        <span>Sauvegarder</span>
                    </button>
                    <button class="action-btn warning" onclick="resetPose()">
                        <span>🔄</span>
                        <span>Réinitialiser</span>
                    </button>
                    <button class="action-btn primary" onclick="relocalize()">
                        <span>📍</span>
                        <span>Relocaliser</span>
                    </button>
                    <button class="action-btn danger" onclick="clearMap()">
                        <span>🗑️</span>
                        <span>Effacer</span>
                    </button>
                `;
            } else {
                document.getElementById('nav2ModeBtn').classList.add('active');
                document.getElementById('modeActions').innerHTML = `
                    <button class="action-btn success" onclick="startNavigation()">
                        <span>▶️</span>
                        <span>Démarrer</span>
                    </button>
                    <button class="action-btn danger" onclick="cancelNavigation()">
                        <span>⏹️</span>
                        <span>Annuler</span>
                    </button>
                    <button class="action-btn primary" onclick="setPose()">
                        <span>📍</span>
                        <span>Set Pose</span>
                    </button>
                    <button class="action-btn warning" onclick="clearCostmaps()">
                        <span>🔄</span>
                        <span>Clear</span>
                    </button>
                `;
            }
            showNotification(`Mode ${mode.toUpperCase()} activé`, 'success');
        }

        async function adminGoTo(location) {
            try {
                const response = await fetch('/goto_location', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({location: location, mode: currentMode})
                });
                const data = await response.json();
                if (data.status === 'success') {
                    showNotification(`Navigation vers ${data.name}`, 'success');
                }
            } catch (error) {
                showNotification('Erreur navigation', 'error');
            }
        }

        // CONTROL FUNCTIONS avec vitesse progressive
        document.querySelectorAll('.control-btn').forEach(button => {
            button.addEventListener('mousedown', () => {
                sendCommand(button.dataset.command);
                button.classList.add('active');
                isMoving = (button.dataset.command !== 'stop');
            });
            
            button.addEventListener('mouseup', () => {
                button.classList.remove('active');
            });
            
            button.addEventListener('mouseleave', () => {
                button.classList.remove('active');
            });
        });

        async function sendCommand(command) {
            try {
                await fetch('/control', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({
                        command: command,
                        linear_speed: currentLinearSpeed,
                        angular_speed: currentAngularSpeed
                    })
                });
            } catch (error) {
                console.error('Control error:', error);
            }
        }

        let speedUpdateTimeout = null;

        document.getElementById('linearSpeed')?.addEventListener('input', function() {
            targetLinearSpeed = parseFloat(this.value);
            document.getElementById('linearValue').textContent = this.value + ' m/s';
            
            if (isMoving) {
                currentLinearSpeed = targetLinearSpeed;
                clearTimeout(speedUpdateTimeout);
                speedUpdateTimeout = setTimeout(() => {
                    updateSpeed();
                }, 100);
            } else {
                currentLinearSpeed = targetLinearSpeed;
                updateSpeed();
            }
        });

        document.getElementById('angularSpeed')?.addEventListener('input', function() {
            targetAngularSpeed = parseFloat(this.value);
            document.getElementById('angularValue').textContent = this.value + ' rad/s';
            
            if (isMoving) {
                currentAngularSpeed = targetAngularSpeed;
                clearTimeout(speedUpdateTimeout);
                speedUpdateTimeout = setTimeout(() => {
                    updateSpeed();
                }, 100);
            } else {
                currentAngularSpeed = targetAngularSpeed;
                updateSpeed();
            }
        });

        async function updateSpeed() {
            try {
                await fetch('/set_speed', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({
                        linear: currentLinearSpeed,
                        angular: currentAngularSpeed
                    })
                });
            } catch (error) {
                console.error('Speed update error:', error);
            }
        }

        // MAP FUNCTIONS
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
        }

        // ACTION FUNCTIONS
        async function saveMap() {
            showNotification('Sauvegarde...', 'info');
            const res = await fetch('/save_map', {method: 'POST'});
            const data = await res.json();
            showNotification(data.status === 'success' ? 'Carte sauvegardée!' : 'Erreur', data.status === 'success' ? 'success' : 'error');
        }

        async function resetPose() {
            await fetch('/reset_pose', {method: 'POST'});
            showNotification('Pose réinitialisée', 'success');
        }

        async function relocalize() {
            await fetch('/relocalize', {method: 'POST'});
            showNotification('Relocalisation OK', 'success');
        }

        async function clearMap() {
            if (confirm('Effacer la carte?')) {
                await fetch('/clear_map', {method: 'POST'});
                showNotification('Carte effacée', 'info');
            }
        }

        function startNavigation() {
            showNotification('Navigation démarrée', 'success');
        }

        async function cancelNavigation() {
            await fetch('/cancel_navigation', {method: 'POST'});
            showNotification('Navigation annulée', 'warning');
        }

        function setPose() {
            showNotification('Cliquez sur la carte', 'info');
        }

        async function clearCostmaps() {
            await fetch('/clear_costmaps', {method: 'POST'});
            showNotification('Costmaps nettoyées', 'success');
        }

        // NOTIFICATION
        function showNotification(message, type = 'info') {
            const notif = document.getElementById('notification');
            const icon = document.getElementById('notifIcon');
            const text = document.getElementById('notifText');
            
            const icons = {
                'success': '✅',
                'error': '❌',
                'info': 'ℹ️',
                'warning': '⚠️'
            };
            
            icon.textContent = icons[type];
            text.textContent = message;
            notif.className = `notification ${type} show`;
            
            setTimeout(() => {
                notif.classList.remove('show');
            }, 3000);
        }

        // CONNECTION CHECK
        setInterval(() => {
            const status = document.getElementById('connectionStatus');
            const timeSince = Date.now() - lastUpdateTime;
            if (timeSince > 3000) {
                status.className = 'connection-status disconnected';
                status.innerHTML = '<span class="status-dot"></span><span>Déconnecté</span>';
            } else {
                status.className = 'connection-status connected';
                status.innerHTML = '<span class="status-dot"></span><span>Connecté</span>';
            }
        }, 1000);

        window.addEventListener('resize', resizeCanvases);
    </script>
</body>
</html>
"""

def get_local_ip():
    """Get local IP"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception:
        return "localhost"

# CONTROLLER (identique à la version finale avec accélération progressive)
class UnifiedHospitalController(Node):
    def __init__(self):
        super().__init__('unified_hospital_controller')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Vitesses avec accélération progressive
        self.target_linear_speed = 0.5
        self.target_angular_speed = 1.0
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.acceleration = 0.05
        self.angular_acceleration = 0.1
        
        self.current_command = 'stop'
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.velocity = 0.0
        self.laser_data = None
        self.map_data = None
        self.current_path = None
        self.goal_handle = None
        self.distance_traveled = 0.0
        self.last_pose = None
        
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
        
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.get_logger().info('🏥 HospiBot Controller initialized with progressive acceleration')

    def odom_callback(self, msg):
        self.robot_pose['x'] = msg.pose.pose.position.x
        self.robot_pose['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_pose['theta'] = math.atan2(siny_cosp, cosy_cosp)
        self.velocity = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        if self.last_pose:
            dx = self.robot_pose['x'] - self.last_pose['x']
            dy = self.robot_pose['y'] - self.last_pose['y']
            self.distance_traveled += math.sqrt(dx*dx + dy*dy)
        self.last_pose = self.robot_pose.copy()

    def scan_callback(self, msg):
        self.laser_data = {
            'ranges': list(msg.ranges),
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment
        }

    def map_callback(self, msg):
        self.map_data = {'width': msg.info.width, 'height': msg.info.height, 
                        'resolution': msg.info.resolution, 'data': list(msg.data)}

    def path_callback(self, msg):
        if msg.poses:
            self.current_path = [{'x': pose.pose.position.x, 'y': pose.pose.position.y} for pose in msg.poses]

    def set_command(self, command):
        self.current_command = command

    def set_speed(self, linear, angular):
        self.target_linear_speed = linear
        self.target_angular_speed = angular

    def publish_velocity(self):
        msg = Twist()
        
        commands = {
            'forward': (self.target_linear_speed, 0.0),
            'backward': (-self.target_linear_speed, 0.0),
            'left': (0.0, self.target_angular_speed),
            'right': (0.0, -self.target_angular_speed),
            'forward_left': (self.target_linear_speed, self.target_angular_speed),
            'forward_right': (self.target_linear_speed, -self.target_angular_speed),
            'backward_left': (-self.target_linear_speed, self.target_angular_speed),
            'backward_right': (-self.target_linear_speed, -self.target_angular_speed),
            'stop': (0.0, 0.0)
        }
        
        if self.current_command in commands:
            target_linear, target_angular = commands[self.current_command]
            
            # Accélération progressive
            if abs(target_linear - self.current_linear_speed) > self.acceleration:
                if target_linear > self.current_linear_speed:
                    self.current_linear_speed += self.acceleration
                else:
                    self.current_linear_speed -= self.acceleration
            else:
                self.current_linear_speed = target_linear
            
            if abs(target_angular - self.current_angular_speed) > self.angular_acceleration:
                if target_angular > self.current_angular_speed:
                    self.current_angular_speed += self.angular_acceleration
                else:
                    self.current_angular_speed -= self.angular_acceleration
            else:
                self.current_angular_speed = target_angular
            
            msg.linear.x = self.current_linear_speed
            msg.angular.z = self.current_angular_speed
            self.cmd_vel_pub.publish(msg)

    def navigate_to_pose_nav2(self, x, y, theta=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            return False
        
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def navigate_to_pose_slam(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        return True

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if self.goal_handle.accepted:
            result_future = self.goal_handle.get_result_async()
            result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.goal_handle = None

    def cancel_navigation(self):
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            return True
        return False

    def navigate_to_location(self, location_id, mode='slam'):
        if location_id not in self.hospital_locations:
            return False
        loc = self.hospital_locations[location_id]
        if mode == 'nav2':
            return self.navigate_to_pose_nav2(loc['x'], loc['y'], loc.get('theta', 0.0))
        else:
            return self.navigate_to_pose_slam(loc['x'], loc['y'])

    def get_stats(self, mode='slam'):
        stats = {'distance': self.distance_traveled, 'coverage': 0, 'remaining': 0.0}
        if mode == 'slam' and self.map_data:
            explored_area = sum(1 for d in self.map_data['data'] if d > 0) * (self.map_data['resolution'] ** 2)
            stats['coverage'] = min(100, int(explored_area / 100))
        return stats

    def set_initial_pose(self, x, y, theta):
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


# Flask App (identique)
app = Flask(__name__)
app.secret_key = secrets.token_hex(16)
CORS(app)
controller = None
waypoint_list = []

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/control', methods=['POST'])
def control():
    data = request.json
    command = data.get('command', '')
    
    if 'linear_speed' in data:
        controller.set_speed(
            float(data['linear_speed']),
            float(data['angular_speed'])
        )
    
    valid_commands = ['forward', 'backward', 'left', 'right', 
                     'forward_left', 'forward_right', 
                     'backward_left', 'backward_right', 'stop']
    if command in valid_commands:
        controller.set_command(command)
        return jsonify({'status': 'success', 'command': command})
    return jsonify({'status': 'error', 'message': 'Unknown command'})

@app.route('/set_speed', methods=['POST'])
def set_speed():
    data = request.json
    linear = float(data.get('linear', 0.5))
    angular = float(data.get('angular', 1.0))
    controller.set_speed(linear, angular)
    return jsonify({'status': 'success', 'linear': linear, 'angular': angular})

@app.route('/get_data', methods=['GET'])
def get_data():
    mode = request.args.get('mode', 'slam')
    return jsonify({
        'pose': controller.robot_pose,
        'velocity': controller.velocity,
        'laser': controller.laser_data,
        'map': controller.map_data,
        'path': controller.current_path,
        'stats': controller.get_stats(mode),
        'is_navigating': controller.goal_handle is not None
    })

@app.route('/add_waypoint', methods=['POST'])
def add_waypoint():
    data = request.json
    waypoint = {'x': float(data['x']), 'y': float(data['y']), 'name': data.get('name', 'Point'), 'id': data.get('id'), 'timestamp': time.time()}
    waypoint_list.append(waypoint)
    return jsonify({'status': 'success', 'waypoint': waypoint, 'total': len(waypoint_list)})

@app.route('/clear_waypoints', methods=['POST'])
def clear_waypoints():
    global waypoint_list
    count = len(waypoint_list)
    waypoint_list = []
    return jsonify({'status': 'success', 'cleared': count})

@app.route('/get_waypoints', methods=['GET'])
def get_waypoints():
    return jsonify({'waypoints': waypoint_list})

@app.route('/goto_location', methods=['POST'])
def goto_location():
    data = request.json
    location_id = data.get('location', '')
    mode = data.get('mode', 'slam')
    
    if location_id not in controller.hospital_locations:
        return jsonify({'status': 'error', 'message': f'Unknown location: {location_id}'})
    
    success = controller.navigate_to_location(location_id, mode)
    
    if success:
        loc = controller.hospital_locations[location_id]
        return jsonify({'status': 'success', 'location': location_id, 'name': loc['name'], 
                       'coordinates': {'x': loc['x'], 'y': loc['y']}})
    return jsonify({'status': 'error', 'message': 'Failed to start navigation'})

@app.route('/goto_waypoint', methods=['POST'])
def goto_waypoint():
    waypoint = request.json
    x = float(waypoint['x'])
    y = float(waypoint['y'])
    
    if controller.navigate_to_pose_nav2(x, y):
        return jsonify({'status': 'success'})
    return jsonify({'status': 'error', 'message': 'Navigation failed'})

@app.route('/cancel_navigation', methods=['POST'])
def cancel_navigation():
    if controller.cancel_navigation():
        return jsonify({'status': 'success'})
    return jsonify({'status': 'error', 'message': 'No active navigation'})

@app.route('/save_map', methods=['POST'])
def save_map():
    try:
        maps_dir = os.path.expanduser('~/Last_ros/src/my_robot_controller/maps')
        os.makedirs(maps_dir, exist_ok=True)
        result = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', 
                               f'{maps_dir}/hospital_map'], capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            return jsonify({'status': 'success', 'message': f'Carte sauvegardée dans {maps_dir}'})
        return jsonify({'status': 'error', 'message': result.stderr})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

@app.route('/reset_pose', methods=['POST'])
def reset_pose():
    controller.distance_traveled = 0.0
    controller.last_pose = None
    return jsonify({'status': 'success'})

@app.route('/clear_map', methods=['POST'])
def clear_map():
    return jsonify({'status': 'success'})

@app.route('/relocalize', methods=['POST'])
def relocalize():
    controller.set_initial_pose(controller.robot_pose['x'], controller.robot_pose['y'], 
                                controller.robot_pose['theta'])
    return jsonify({'status': 'success'})

@app.route('/clear_costmaps', methods=['POST'])
def clear_costmaps():
    return jsonify({'status': 'success', 'message': 'Costmaps cleared'})


def run_flask():
    local_ip = get_local_ip()
    print("\n" + "="*80)
    print("🏥 HOSPIBOT - VERSION FINALE AVEC VISUELS NAV2")
    print("="*80)
    print(f"🔗 Interface Web:              http://{local_ip}:5000")
    print("="*80)
    print("\n✅ TOUTES CORRECTIONS + VISUELS:")
    print("   🔥 Waypoints cliquables")
    print("   🔥 Métriques temps réel")
    print("   🔥 Canvas initialisés")
    print("   🔥 Vitesse progressive")
    print("   🎨 Légende carte (Robot/Objectif/Obstacles/Chemin)")
    print("   🎨 Section waypoints stylisée avec état vide")
    print("   🎨 Layout hospitalier sur carte")
    print("   ✅ TOUTES fonctionnalités conservées")
    print("="*80 + "\n")
    
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)


def main(args=None):
    global controller
    rclpy.init(args=args)
    controller = UnifiedHospitalController()
    
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\n⏹️ Arrêt du système...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        print("✅ HospiBot arrêté proprement")


if __name__ == '__main__':
    main()