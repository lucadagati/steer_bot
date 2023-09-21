#!/bin/bash

# Cartelle da spostare
folders=("cone")

# Directory di destinazione
dest_folder="$HOME/.gazebo/models"

# Crea la directory di destinazione se non esiste
mkdir -p $dest_folder

# Sposta le cartelle
for folder in "${folders[@]}"; do
  if [ -d "$folder" ]; then
    mv $folder $dest_folder/
    echo "Spostamento di $folder completato."
  else
    echo "La cartella $folder non esiste."
  fi
done

echo "Finito."
