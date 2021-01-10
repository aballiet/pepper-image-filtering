# Etudes de l'impact de la compression sur la capacité de transmission et sur les ressources de calcul

## Introduction

En effet, pendant des compétitions comme la robocup, la qualité du réseau sans fil devient un facteur critique pour la réception des données
afin de réaliser les traitements à distance. Il est donc nécessaire d'optimiser la transmission, en plus du filtrage, notre objectif est de voir si la compression 
des images filtrés a un impact important pour encore plus économiser de bande passante, si oui, quelle méthode et quels paramètres sont les plus optimaux.

## Déscription du projet
Ce dossier contients trois scripts python:
- auto_compression.py :

  Mesurer la fréquence de la réception sur la machine à distance avec différentes méthodes de compression et des différentes niveaux, 
  géner automatiquement des fichiers csv en sortie.
  
- csvPlot.py:

  Plot les fichiers csv avec le format approprié automatiquement et les sauvegarder sous format png.

- compressionCPUMonitor.py :

  Mesurer l'impact sur les ressources de calcul de la compression et de la décompression sur le robot et sur la machine à distance.


##
