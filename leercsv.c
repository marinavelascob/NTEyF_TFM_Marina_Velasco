#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_COLUMNAS 8
#define MAX_FILAS 1000
#define MAX_LONGITUD_LINEA 1024

int main() {
    char* archivo_csv = "datos.csv";
    FILE* archivo = fopen(archivo_csv, "r");
    if (archivo == NULL) {
        printf("No se pudo abrir el archivo '%s'.\n", archivo_csv);
        return 1;
    }

    char linea[MAX_LONGITUD_LINEA];
    char* token;
    int fila = 0;
    int columna = 0;
    int tasa_BA[MAX_FILAS], distancia_BA[MAX_FILAS], tasa_BB[MAX_FILAS], distancia_BB[MAX_FILAS];
    int tasa_LA[MAX_FILAS], distancia_LA[MAX_FILAS], tasa_LB[MAX_FILAS], distancia_LB[MAX_FILAS];

    while (fgets(linea, MAX_LONGITUD_LINEA, archivo) != NULL) {
        if (fila == 0) {
            // Saltar la primera fila (encabezado).
            fila++;
            continue;
        }

        token = strtok(linea, ",");
        columna = 0;
        while (token != NULL && columna < MAX_COLUMNAS) {
            switch (columna) {
                case 0:
                    distancia_BA[fila - 1] = atof(token);
                    printf("distancia_ba=%d\n",distancia_BA[fila-1]);
                    break;
                case 1:
                    tasa_BA[fila - 1] = atof(token);
                    break;
                case 2:
                    distancia_BB[fila - 1] = atof(token);
                    break;
                case 3:
                    tasa_BB[fila - 1] = atof(token);
                    break;
                case 4:
                    distancia_LA[fila - 1] = atof(token);
                    break;
                case 5:
                    tasa_LA[fila - 1] = atof(token);
                    break;
                case 6:
                    distancia_LB[fila - 1] = atof(token);
                    break;
                case 7:
                    tasa_LB[fila - 1] = atof(token);
                    printf("TASALB=%d\n",tasa_LB[fila-1]);
                    break;
                default:
                    break;
            }

            token = strtok(NULL, ",");
            columna++;
        }

        fila++;
    }

    fclose(archivo);

    // Imprimir los datos para verificar que se hayan leído correctamente.
    
    for (int i = 0; i < fila - 1; i++) {
        printf("%d, %d, %d, %d, %d, %d, %d, %d\n", 
               distancia_BA[i], tasa_BA[i], distancia_BB[i], tasa_BB[i],
               distancia_LA[i], tasa_LA[i], distancia_LB[i], tasa_LB[i]);
    }

    return 0;
}
