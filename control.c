#include <stdio.h>
#include <stdlib.h>

int main()
{
	int modo = 0;
	printf("\n¡Bienvenido al programa para el uso de la máquina de modelado análogo!\n");
	
	while(modo!=3){
		printf("\nEstá en el menú principal.\nSeleccione el submenú al que desea acceder:\n");
		printf("1. Herramientas de movimiento de motores.\n");
		printf("2. Experimento combinado.\n");
		printf("3. Apagar.\n");
		scanf("%d",&modo);
		printf("modo %d", modo);
		if(modo==1){
			system("./herramientas");
			}
		if(modo==2){
			system("./movimiento");
			}
	}
	
	printf("¡Fin del experimento!\n");
	return 0;
}
