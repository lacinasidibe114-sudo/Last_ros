#!/bin/bash
# Script pour sauvegarder la carte

MAP_NAME=${1:-hospital_map}
SAVE_DIR=~/Last_ros/maps

mkdir -p $SAVE_DIR

echo "Sauvegarde de la carte: $MAP_NAME"
ros2 run nav2_map_server map_saver_cli -f $SAVE_DIR/$MAP_NAME

echo "Carte sauvegardée dans: $SAVE_DIR/$MAP_NAME.pgm et $MAP_NAME.yaml"