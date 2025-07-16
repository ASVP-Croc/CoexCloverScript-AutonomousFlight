# CoexCloverScript-AutonomousFlight

Script per il volo autonomo del drone Coex Clover 4 in scenario SAFE. Il codice ottimizza la scansione ed il percorso del drone su un'area data.

Lo script richiede all'utente di inserire le coordinate di 4 vertici per identificare il poligono da scansionare. Viene quindi generata una griglia esagonale sulla base del raggio di scansione del drone e da qui vengono estratti i punti interni al poligono per ottenere solo punti di scansione necessari. Il codice ottimizza il percorso dei punti sui quali si sposterà il drone per minimizzare il tempo in volo. Vengono utilizzati i servizi ROS per la comunicazione tra Raspberry Pi e il controller di volo al fine di far eseguire la scansione in maniera autonoma.

NOTA!
Il drone Coex Clover 4 utilizza come riferimento per il sistema di coordinate la posizione iniziale dove viene acceso (coordinate x=0, y=0, z=0).
