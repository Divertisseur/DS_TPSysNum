# DS_TPSysNum
Le ds de systèmes numériques

L'adresse du capteur en I2C est 0x48 et le registre pour récupérer la température est 0x00.

On procède tout d'abord à un test du capteur pour savoir si il communique correctement.
Puis, on utilise la fonction HAL_Master_Transmit et HAL_Master_Receive pour récupérer la température.
On effectue ensuite une conversion en flottant en divisant par 16 comme précisé dans la documentation, afin de récupérer la vraie valeur de la température.
