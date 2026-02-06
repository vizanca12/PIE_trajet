# Simulateur de Navigation Autonome BlueBoat 🛥️
Ce projet est un environnement de test virtuel développé en Python et Pygame pour le BlueBoat. Il permet de valider les algorithmes de navigation et d'évitement d'obstacles avant leur déploiement sur le bateau réel.

## 🚀 Fonctionnalités Principales
* Génération de Trajectoire : Création automatique d'une route en "8" (lemniscate) entre deux points de repère (bouées).

* Planification d'Entrée : Le bateau calcule automatiquement le meilleur point pour rejoindre la trajectoire établie.

* Évitement d'Obstacles : Système de détection de proximité qui calcule une trajectoire de déviation en temps réel pour éviter les collisions.

* Gestion des États : Le système suit un cycle de mission allant de la recherche de la route jusqu'à l'arrivée finale au point de destination.

## 📂 Organisation du Code

**blueboat_model.py**
* Contient toute la logique physique et décisionnelle :

    *  Calculs Vectoriels : Gestion des positions et des directions via une classe Vector2.

    * Intelligence de Navigation : Algorithmes de suivi de points (Lookahead) et calcul des forces d'évitement.

**test_scenario.py**
* Gère l'interface et l'exécution de la simulation :

    * Moteur Graphique : Rendu visuel à 60 FPS avec Pygame.

    * Interface Utilisateur (HUD) : Affichage de la télémétrie, du progrès de la mission et des capteurs de proximité.

## 🔜 Prochaines Étapes
* L'objectif final de ce simulateur est de servir de base logicielle pour le bateau réel :

    * Communication avec NVIDIA Jetson : Portage du code pour qu'il devienne le module de navigation principal embarqué sur la Jetson du BlueBoat.

    * Pilotage Réel : Conversion des vecteurs de direction calculés ici en commandes réelles pour les moteurs via une liaison série.

    * Intégration ROS2 : Standardisation des données pour une utilisation dans un environnement robotique complet.

## 🛠️ Installation et Lancement
Installez les dépendances : pip install pygame

Activez votre environnement virtuel : source venv/bin/activate

Lancez le simulateur :

Bash
```
python src/test_scenario.py
```
