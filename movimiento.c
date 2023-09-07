#include <wiringPi.h>
#include <pthread.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int EN = 25; //ENABLE DE TODOS LOS MOTORES (GPIO26)

//BASE A [BA] (MOTOR 1)
int DIR_BA = 6; //(GPIO25)
int STEP_BA = 24; //(GPIO19)
int pos_BA;

//BASE B [BB] (MOTOR 2)
int DIR_BB = 4; //(GPIO23)
int STEP_BB = 1; //(GPIO18)
int pos_BB;

//LADO A [LA] (MOTORES 3 Y 4)
int DIR_LA = 27; //(GPIO16)
int STEP_LA = 23; //(GPIO13)
int pos_LA;

//LADO B [LB] (MOTORES 5 Y 6)
int DIR_LB = 5; //(GPIO24)
int STEP_LB = 26; //(GPIO12)
int pos_LB;

//INTERRUPTORES DE FIN DE CARRERA (MULTIPLEXOR DCBA)
int SALIDA_INT = 21; //Salida del multiplexor (GPIO5)
int A_SELECT_INT = 0; //Entrada multiplexor (GPIO17)
int B_SELECT_INT = 2; //Entrada multiplexor (GPIO27)
int C_SELECT_INT = 3; //Entrada multiplexor (GPIO22)
int D_SELECT_INT = 22; //Entrada multiplexor (GPIO6)

FILE *posiciones; //Archivo de posiciones

int k = 25600; // Número de pasos en los que se divide una vuelta
int f0 = 15000; //15 kHz para los posicionamientos rápidos

int int_BA_m, int_BA_nm, int_BB_m, int_BB_nm, int_LA_m, int_LA_nm, int_LB_m, int_LB_nm, int_danger;
int i_BA, i_BB, i_LA, i_LB, n_pasos_BA, n_pasos_BB, n_pasos_LA, n_pasos_LB;
int tdelay_BA, tdelay_BB, tdelay_LA, tdelay_LB, sgn_BA, sgn_BB, sgn_LA, sgn_LB, count_BA, count_BB, count_LA, count_LB;
int num_mot;

int reset;
int problema=0;

//Definimos las variables para el movimiento
int d_BA=-1000;
int d_BB=-1000;
int d_LA=-1000;
int d_LB=-1000;
int tasa_BA=-1000;
int tasa_BB=-1000;
int tasa_LA=-1000;
int tasa_LB=-1000;

//Definimos variables para leer csv
#define MAX_COLUMNAS 8
#define MAX_FILAS 1000
#define MAX_LONGITUD_LINEA 1024

// Definimos los pines de los encoders
#define PIN_ENCODER_BA 30 //(GPIO0)
#define PIN_ENCODER_BB 31 //(GPIO1)
#define PIN_ENCODER_LA 15 //(GPIO14)
#define PIN_ENCODER_LB 16 //(GPIO15)

// Variable para almacenar el conteo de pulsos
int pulso_BA = 0;
int pulso_BB = 0;
int pulso_LA = 0;
int pulso_LB = 0;


//Función para medir el encoder de la base A
PI_THREAD(Medir_encoder_BA){
	pulso_BA=0;
	int fase_pulso_BA=0;
	for(;;){
		if ((digitalRead(PIN_ENCODER_BA)==HIGH)&&(fase_pulso_BA=0)){
			pulso_BA++;
			fase_pulso_BA=1;
			while(digitalRead(PIN_ENCODER_BA)==HIGH){}
		}
		else if ((digitalRead(PIN_ENCODER_BA)==LOW)&&(fase_pulso_BA=1)){
			pulso_BA++;
			fase_pulso_BA=0;
			while(digitalRead(PIN_ENCODER_BA)==LOW){}}
		delayMicroseconds(600);
		}	
}


//Función para medir el encoder de la base B
PI_THREAD(Medir_encoder_BB){
	pulso_BB=0;
	int fase_pulso_BB=0;
	for(;;){
		if ((digitalRead(PIN_ENCODER_BB)==HIGH)&&(fase_pulso_BB=0)){
			pulso_BB++;
			fase_pulso_BB=1;
			while(digitalRead(PIN_ENCODER_BB)==HIGH){}
		}
		else if ((digitalRead(PIN_ENCODER_BB)==LOW)&&(fase_pulso_BB=1)){
			pulso_BB++;
			fase_pulso_BB=0;
			while(digitalRead(PIN_ENCODER_BB)==LOW){}}
		delayMicroseconds(600);
		}	
}


//Función para medir el encoder del lado A
PI_THREAD(Medir_encoder_LA){
	pulso_LA=0;
	int fase_pulso_LA=0;
	for(;;){
		if ((digitalRead(PIN_ENCODER_LA)==HIGH)&&(fase_pulso_LA=0)){
			pulso_LA++;
			fase_pulso_LA=1;
			while(digitalRead(PIN_ENCODER_LA)==HIGH){}
		}
		else if ((digitalRead(PIN_ENCODER_LA)==LOW)&&(fase_pulso_LA=1)){
			pulso_LA++;
			fase_pulso_LA=0;
			while(digitalRead(PIN_ENCODER_LA)==LOW){}}
		delayMicroseconds(600);
		}	
}


//Función para medir el encoder del lado B
PI_THREAD(Medir_encoder_LB){
	pulso_LB=0;
	int fase_pulso_LB=0;
	for(;;){
		if ((digitalRead(PIN_ENCODER_LB)==HIGH)&&(fase_pulso_LB=0)){
			pulso_LB++;
			fase_pulso_LB=1;
			while(digitalRead(PIN_ENCODER_LB)==HIGH){}
		}
		else if ((digitalRead(PIN_ENCODER_LB)==LOW)&&(fase_pulso_LB=1)){
			pulso_LB++;
			fase_pulso_LB=0;
			while(digitalRead(PIN_ENCODER_LB)==LOW){}}
		delayMicroseconds(600);
		}	
}


// Función para monitorizar la lectura de las entradas del multiplexor
PI_THREAD(Lectura_interruptores){
	int_BA_m = 0;
	int_BA_nm = 0;
	int_BB_m = 0;
	int_BB_nm = 0;
	int_LA_m = 0;
	int_LA_nm = 0;
	int_LB_m = 0;
	int_LB_nm = 0;
	int_danger =0;

	for(;;){
		int g = 0;
		int j = 0;
		int i;
		int valor[12];

		for (i=0;i<12;i++){
			valor[i]=0;
		}

		while(g<4){
			while (j<12){
				delay(20);
				if (j % 2 == 0) {
					digitalWrite(A_SELECT_INT, LOW);} 
				else {
					digitalWrite(A_SELECT_INT, HIGH);}

				if(j<8){
					digitalWrite(D_SELECT_INT, LOW);
					if(j<4){
						digitalWrite(C_SELECT_INT, LOW);
						if(j<2){
							digitalWrite(B_SELECT_INT, LOW);}
						else{
							digitalWrite(B_SELECT_INT, HIGH);}
					}
					else{
						digitalWrite(C_SELECT_INT, HIGH);
						if(j<6){
							digitalWrite(B_SELECT_INT, LOW);}
						else{
							digitalWrite(B_SELECT_INT, HIGH);}
					}
				}
				else{
					digitalWrite(D_SELECT_INT, HIGH);
					digitalWrite(C_SELECT_INT, LOW);
					if(j<10){
						digitalWrite(B_SELECT_INT, LOW);}
					else{
						digitalWrite(B_SELECT_INT, HIGH);}
				}
				delay(20);
				valor[j] += digitalRead(SALIDA_INT);
				j++;
			}
			j=0;
			g++;
		}

		if(valor[0]==4){
			int_BA_m = 1;
			pos_BA=0;
			//printf("BASE A (Motor 1) en extremo motor.\n");
			if(reset==0){n_pasos_BA = -1;}
		}
		else{int_BA_m = 0;}

		if(valor[1]==4){
			int_BA_nm = 1;
			pos_BA=200;
			//printf("BASE A (Motor 1) en extremo no motor.\n");
			if(reset==0){n_pasos_BA = -1;}
		}
		else {int_BA_nm = 0;}

		if(valor[2]==4){
			int_BB_m = 1;
			pos_BB=0;
			//printf("BASE B (Motor 2) en extremo motor.\n");
			if(reset==0){n_pasos_BB = -1;}
		}
		else {int_BB_m = 0;}

		if(valor[3]==4){
			int_BB_nm = 1;
			pos_BB=200;
			//printf("BASE B (Motor 2) en extremo no motor.\n");
			if(reset==0){n_pasos_BB = -1;}
		}
		else {int_BB_nm = 0;}

		if((valor[4]==4)||(valor[6]==4)){
			int_LA_m = 1;
			pos_LA=0;
			//printf("PARED A (Motores 3 y 4) en extremo motor.\n");
			if(reset==0){n_pasos_LA = -1;}
		}
		else{int_LA_m = 0;}

		if((valor[5]==4)||(valor[7]==4)){
			int_LA_nm = 1;
			pos_LA=200;
			//printf("PARED A (Motores 3 y 4) en extremo no motor.\n");
			if(reset==0){n_pasos_LA = -1;}
		}
		else {int_LA_nm = 0;}

		if((valor[8]==4)||(valor[10]==4)){
			int_LB_m = 1;
			pos_LB=0;
			//printf("PARED B (Motores 5 y 6) en extremo motor.\n");
			if(reset==0){n_pasos_LB = -1;}
		}
		else{int_LB_m = 0;}

		if((valor[9]==4)||(valor[11]==4)){
			int_LB_nm = 1;
			pos_LB=200;
			//printf("PARED B (Motores 5 y 6) en extremo no motor.\n");
			if(reset==0){n_pasos_LB = -1;}
		}
		else {int_LB_nm = 0;}
		int_danger = int_BA_m+int_BA_nm+int_BB_m+int_BB_nm+int_LA_m+int_LA_nm+int_LB_m+int_LB_nm;
		//printf("interruptores= %i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i\n", valor[0], valor[1], valor[2], valor[3], valor[4], valor[5], valor[6], valor[7], valor[8], valor[9], valor[10], valor[11]);
	}
}


//Función para mover el motor de la base A cada paso y reescribir posición
PI_THREAD(Motor_BA){
	n_pasos_BA = 0; //PREGUNTAR POR ESTO
	for(;;){
		i_BA = 0;
		while(i_BA<n_pasos_BA){
			digitalWrite(STEP_BA, LOW);
			delayMicroseconds(tdelay_BA);
			digitalWrite(STEP_BA, HIGH);
			delayMicroseconds(tdelay_BA);
			i_BA++;
			if (i_BA % 12800 == 0) {
				pos_BA = pos_BA + sgn_BA;
			}
		}
		delay(500);
	}
}


//Función para mover el motor de la base B cada paso y reescribir posición
PI_THREAD(Motor_BB) {
	n_pasos_BB = 0; //PREGUNTAR POR ESTO
	for (;;) {
		i_BB = 0;
		while (i_BB < n_pasos_BB) {
			digitalWrite(STEP_BB, LOW);
			delayMicroseconds(tdelay_BB);
			digitalWrite(STEP_BB, HIGH);
			delayMicroseconds(tdelay_BB);
			i_BB++;
			if (i_BB % 12800 == 0) {
				pos_BB = pos_BB + sgn_BB;
			}
		}
		delay(500);
	}
}


//Función para mover el motor del lado A cada paso y reescribir posición
PI_THREAD(Motor_LA){
	n_pasos_LA = 0; //PREGUNTAR POR ESTO
	for(;;){
		i_LA = 0;
		while(i_LA<n_pasos_LA){
			digitalWrite(STEP_LA, LOW);
			delayMicroseconds(tdelay_LA);
			digitalWrite(STEP_LA, HIGH);
			delayMicroseconds(tdelay_LA);
			i_LA++;
			if (i_LA % 12800 == 0) {
				pos_LA = pos_LA + sgn_LA;
			}
		}
		delay(500);
	}
}


//Función para mover el motor del lado B cada paso y reescribir posición
PI_THREAD(Motor_LB) {
	n_pasos_LB = 0; //PREGUNTAR POR ESTO
	for (;;) {
		i_LB = 0;
		while (i_LB < n_pasos_LB) {
			digitalWrite(STEP_LB, LOW);
			delayMicroseconds(tdelay_LB);
			digitalWrite(STEP_LB, HIGH);
			delayMicroseconds(tdelay_LB);
			i_LB++;
			if (i_LB % 12800 == 0) {
				pos_LB = pos_LB + sgn_LB;
			}
		}
		delay(500);
	}
}


//Función para guardar la posición en el fichero
PI_THREAD(Guardado_Posicion) {
	int j;
	for (;;) {
		while ((n_pasos_BA > 0) || (n_pasos_BB > 0) || (n_pasos_LA > 0) || (n_pasos_LB > 0)) {
			delay(500);
			posiciones = fopen("posiciones.txt", "w");
			while (posiciones == NULL) { //Por si hubiera ocurrido un error creando el archivo
				posiciones = fopen("posiciones.txt", "w");
			}
			fprintf(posiciones, "%i,%i,%i,%i", pos_BA, pos_BB, pos_LA, pos_LB);
			fclose(posiciones);
			delay(500);
			j=1;
		}
		if(j==1){
			posiciones = fopen("posiciones.txt", "w");
			while (posiciones == NULL) { //Por si hubiera ocurrido un error creando el archivo
				posiciones = fopen("posiciones.txt", "w");
			}
			fprintf(posiciones, "%i,%i,%i,%i", pos_BA, pos_BB, pos_LA, pos_LB);
			fclose(posiciones);
		}
		j=0;
	}
}


//Función que almacena el valor de la posición en data y luego lee la posición
void lectura_posicion(){
	char data[15];
	posiciones = fopen("posiciones.txt","r"); 
	while(posiciones==NULL){ //Por si hubiera ocurrido un error creando el archivo
		posiciones = fopen("posiciones.txt","r"); 
	}
	 
	int j=0;
	while(!feof(posiciones)){
		fread(&data[j],sizeof(char),1,posiciones);
		//printf("%c %i\n",data[j],j);
		j++;
	}
	fclose(posiciones);
	
	sscanf(data, "%d,%d,%d,%d\n", &pos_BA, &pos_BB, &pos_LA, &pos_LB);

	if (pos_BA < 0 || pos_BA>400){
		printf("ERROR FATAL. POR FAVOR, REALICE UN RESETEO.");
	}
	if (pos_BB < 0 || pos_BB>400) {
		printf("ERROR FATAL. POR FAVOR, REALICE UN RESETEO.");
	}
	if (pos_LA < 0 || pos_LA>200){
		printf("ERROR FATAL. POR FAVOR, REALICE UN RESETEO.");
	}
	if (pos_LB < 0 || pos_LB>200) {
		printf("ERROR FATAL. POR FAVOR, REALICE UN RESETEO.");
	}
}


//Función que permite sacar a los motores de la posición extrema
void rutina_interruptores(){
	printf("Comienzo de la rutina para sacar a los motores del extremo.\n");
	tdelay_BA = 1000*1000/(2*500);
	tdelay_BB = 1000*1000/(2*500);
	tdelay_LA = 1000*1000/(2*500);
	tdelay_LB = 1000*1000/(2*500);

	int pos_motor;

	// Motor 1 en extremo
	if((int_BA_nm==1)||(int_BA_m==1)){
		if(int_BA_m==1){ //extremo motor
			digitalWrite(DIR_BA,HIGH);
			pos_motor=0;
			delay(20);
		}
		else if(int_BA_nm==1){ //extremo no motor
			digitalWrite(DIR_BA,LOW);
			pos_motor=1;
			delay(20);
		}
		reset=1;
		n_pasos_BA = 10000;

		while((int_BA_nm==1)||(int_BA_m==1)){
			i_BA = 0;
			printf("BASE A.\n");
			printf("interruptor motor=%i, interruptor no motor=%i\n", int_BA_m, int_BA_nm);
		}

		reset=0;
		n_pasos_BA = -1;
			
		//Por si acaso
		digitalWrite(STEP_BA, HIGH);
		delay(20);
			
		//Todo a cero por si acaso
		digitalWrite(DIR_BA, HIGH);
		digitalWrite(EN, HIGH);
			
		if(pos_motor==0){
			pos_BA = 0;
		}
		else if(pos_motor==1){
			pos_BA = 200;
		}
	}

	// Motor 2 en extremo
	if((int_BB_nm==1)||(int_BB_m==1)){ //extremo motor
		if(int_BB_m==1){
			digitalWrite(DIR_BB,HIGH);
			pos_motor=0;
			delay(20);
		}
		else if(int_BB_nm==1){ //extremo no motor
			digitalWrite(DIR_BB,LOW);
			pos_motor=1;
			delay(20);
		}
		reset=1;
		n_pasos_BB = 10000;

		while((int_BB_nm==1)||(int_BB_m==1)){
			i_BB = 0;
			printf("BASE B.\n");
			printf("interruptor motor=%i, interruptor no motor=%i\n", int_BB_m, int_BB_nm);
		}

		reset=0;
		n_pasos_BB = -1;
			
		//Por si acaso
		digitalWrite(STEP_BB, HIGH);
		delay(20);
			
		//Todo a cero por si acaso
		digitalWrite(DIR_BB, HIGH);
		digitalWrite(EN, HIGH);
			
		if(pos_motor==0){
			pos_BB = 0;
		}
		else if(pos_motor==1){
			pos_BB = 200;
		}
	}


	// Motores 3 y 4 en extremo
	if((int_LA_nm==1)||(int_LA_m==1)){ //extremo motor
		if(int_LA_m==1){
			digitalWrite(DIR_LA,LOW);
			pos_motor=0;
			delay(20);
		}
		else if(int_LA_nm==1){ //extremo no motor
			digitalWrite(DIR_LA,HIGH);
			pos_motor=1;
			delay(20);
		}
		reset=1;
		n_pasos_LA = 10000;

		while((int_LA_nm==1)||(int_LA_m==1)){
			i_LA = 0;
			printf("LADO B.\n");
			printf("interruptor motor=%i, interruptor no motor=%i\n", int_LA_m, int_LA_nm);
		}

		reset=0;
		n_pasos_LA = -1;
			
		//Por si acaso
		digitalWrite(STEP_LA, HIGH);
		delay(20);
			
		//Todo a cero por si acaso
		digitalWrite(DIR_LA, HIGH);
		digitalWrite(EN, HIGH);
			
		if(pos_motor==0){
			pos_LA = 0;
		}
		else if(pos_motor==1){
			pos_LA = 200;
		}
	}

	// Motores 5 y 6 en extremo
	if((int_LB_nm==1)||(int_LB_m==1)){ //extremo motor
		if(int_LB_m==1){
			digitalWrite(DIR_LB,HIGH);
			pos_motor=0;
			delay(20);
		}
		else if(int_LB_nm==1){ //extremo no motor
			digitalWrite(DIR_LB,LOW);
			pos_motor=1;
			delay(20);
		}
		reset=1;
		n_pasos_LB = 10000;

		while((int_LB_nm==1)||(int_LB_m==1)){
			i_LB = 0;
			printf("LADO B.\n");
			printf("interruptor motor=%i, interruptor no motor=%i\n", int_LB_m, int_LB_nm);
		}

		reset=0;
		n_pasos_LB = -1;
			
		//Por si acaso
		digitalWrite(STEP_LB, HIGH);
		delay(20);
			
		//Todo a cero por si acaso
		digitalWrite(DIR_LB, HIGH);
		digitalWrite(EN, HIGH);
			
		if(pos_motor==0){
			pos_LB = 0;
		}
		else if(pos_motor==1){
			pos_LB = 200;
		}
	}
	printf("Final de la rutina para sacar a los motores del extremo.\n");
}


//Función que permite desplazar los motores a la vez en las fases
void movimiento(){
	void piLock();
	pulso_BA = 0;
	pulso_BB = 0;
	pulso_LA = 0;
	pulso_LB = 0;
	void piUnLock();
	
	float d_pulsos_BA=0;
	float d_pulsos_BB=0;
	float d_pulsos_LA=0;
	float d_pulsos_LB=0;

	while (((d_BA + pos_BA) < 0) || ((d_BA + pos_BA) > 420)) {
	printf("No se puede efectuar el movimiento en la base A porque nos salimos de rango\n");
	}
	while (((d_BB + pos_BB) < 0) || ((d_BB + pos_BB) > 420)) {
	printf("No se puede efectuar el movimiento en la base B porque nos salimos de rango\n");
	}
	while (((d_LA + pos_LA) < 0) || ((d_LA + pos_LA) > 200)) {
	printf("No se puede efectuar el movimiento en la pared A porque nos salimos de rango\n");
	}
	while (((d_LB + pos_LB) < 0) || ((d_LB + pos_LB) > 200)) {
	printf("No se puede efectuar el movimiento en la pared B porque nos salimos de rango\n");
	}
	while ((tasa_BA<0)||(tasa_BA>20000)) {
		printf("No se puede efectuar el movimiento en la base A porque la tasa de deformación no se encuentra entre los límites 1-20000 mm/h.\n");
	}
	while ((tasa_BB<0)||(tasa_BB>20000)) {
		printf("No se puede efectuar el movimiento en la base A porque la tasa de deformación no se encuentra entre los límites 1-20000 mm/h.\n");
	}
	while ((tasa_LA<0)||(tasa_LA>20000)) {
		printf("No se puede efectuar el movimiento en la base A porque la tasa de deformación no se encuentra entre los límites 1-20000 mm/h.\n");
	}
	while ((tasa_LB<0)||(tasa_LB>20000)) {
		printf("No se puede efectuar el movimiento en la base A porque la tasa de deformación no se encuentra entre los límites 1-20000 mm/h.\n");
	}

	if (tasa_BA!=0){
		printf("tasa_BA=%d\n", tasa_BA);
		int f_BA = tasa_BA * k / (2 * 3600); //frecuencia en s^-1
		tdelay_BA = 1000*1000/(2*f_BA);
		}
	if (tasa_BB!=0){
		printf("tasa_BB=%d\n", tasa_BB);
		int f_BB = tasa_BB * k / (2 * 3600); //frecuencia en s^-1
		tdelay_BB = 1000*1000/(2*f_BB);
		}
	if (tasa_LA!=0){
		printf("tasa_LA=%d\n", tasa_LA);
		int f_LA = tasa_LA * k / (2 * 3600); //frecuencia en s^-1
		tdelay_LA = 1000*1000/(2*f_LA);
		}
	if (tasa_LB!=0){
		printf("tasa_LB=%d\n", tasa_LB);
		int f_LB = tasa_LB * k / (2 * 3600); //frecuencia en s^-1
		tdelay_LB = 1000*1000/(2*f_LB);
		}
	
	// Habilito todos los motores
	digitalWrite(EN, LOW);
	delay(200);

	// EStablecemos laa direcciones de los motores
	if(d_BA<0){ //Negativa es hacia motor
		digitalWrite(DIR_BA, LOW);
		sgn_BA = -1;
	}
	if(d_BA>0){ //Positiva es hacia no motor
		digitalWrite(DIR_BA, HIGH);
		sgn_BA = 1;
	}
	if(d_BB<0){ //Negativa es hacia motor
		digitalWrite(DIR_BB, LOW);
		sgn_BB = -1;
	}
	if(d_BB>0){ //Positiva es hacia no motor
		digitalWrite(DIR_BB, HIGH);
		sgn_BB = 1;
	}
	if(d_LA<0){ //Negativa es hacia motor
		digitalWrite(DIR_LA, HIGH);
		sgn_LA = -1;
	}
	if(d_LA>0){ //Positiva es hacia no motor
		digitalWrite(DIR_LA, LOW);
		sgn_LA = 1;
	}
	if(d_LB<0){ //Negativa es hacia motor
		digitalWrite(DIR_LB, LOW);
		sgn_LB = -1;
	}
	if(d_LB>0){ //Positiva es hacia no motor
		digitalWrite(DIR_LB, HIGH);
		sgn_LB = 1;
	}

	delay(200);
		
	n_pasos_BA = fabs(d_BA)*k/2;
	n_pasos_BB = fabs(d_BB)*k/2;
	n_pasos_LA = fabs(d_LA)*k/2;
	n_pasos_LB = fabs(d_LB)*k/2;

	while (((i_BA<n_pasos_BA)||(i_BB<n_pasos_BB)||(i_LA<n_pasos_LA)||(i_LB<n_pasos_LB))&&(int_danger<1)){
		delay(20);
		if (i_BA >= n_pasos_BA) {
			n_pasos_BA=-1;
		}
		if (i_BB >= n_pasos_BB) {
			n_pasos_BB=-1;
		}
		if (i_LA >= n_pasos_LA) {
			n_pasos_LA=-1;
		}
		if (i_LB >= n_pasos_LB) {
			n_pasos_LB=-1;
		}
	}
	n_pasos_BA=-1;
	n_pasos_BB=-1;
	n_pasos_LA=-1;
	n_pasos_LB=-1;

	d_pulsos_BA=pulso_BA*2/500; //DISTANCIA RECORRIDA SEGUN PULSOS DEL ENCODER
	d_pulsos_BB=pulso_BB*2/500; //DISTANCIA RECORRIDA SEGUN PULSOS DEL ENCODER
	d_pulsos_LA=pulso_LA*2/500; //DISTANCIA RECORRIDA SEGUN PULSOS DEL ENCODER
	d_pulsos_LB=pulso_LB*2/500; //DISTANCIA RECORRIDA SEGUN PULSOS DEL ENCODER

	//PONEMOS TODO A 0
	digitalWrite (DIR_BA, HIGH);
	digitalWrite (STEP_BA, HIGH);
	
	digitalWrite (DIR_BB, HIGH);
	digitalWrite (STEP_BB, HIGH);

	digitalWrite (DIR_LA, HIGH);
	digitalWrite (STEP_LA, HIGH);
	
	digitalWrite (DIR_LB, HIGH);
	digitalWrite (STEP_LB, HIGH);
	
	delay(200);
	digitalWrite (EN, HIGH);
	printf("Enable off \n");

	if (int_danger>0){
		// Si llegamos al interruptor antes de tiempo
		printf("WARNING: MOTOR EN EXTREMO DEL RECORRIDO");
		rutina_interruptores();
		final=1;
		problema=1;
	}
	else{
		// Comprobamos con el encoder si se mueve lo que configuramos
		float error_relativo_BA=0;
		float error_relativo_BB=0;
		float error_relativo_LA=0;
		float error_relativo_LB=0;
		error_relativo_BA=fabs(fabs(d_BA)-d_pulsos_BA)*100/fabs(d_BA);
		error_relativo_BB=fabs(fabs(d_BB)-d_pulsos_BB)*100/fabs(d_BB);
		error_relativo_LA=fabs(fabs(d_LA)-d_pulsos_LA)*100/fabs(d_LA);
		error_relativo_BB=fabs(fabs(d_BB)-d_pulsos_BB)*100/fabs(d_LB);
		if ((error_relativo_BA>5)||(error_relativo_BB>5)||(error_relativo_LA>5)||(error_relativo_LB>5)){
			printf("WARNING: Error relativo de distancias medidas con encoder mayor del 10 porciento\n");
		}
		printf ("Distancias recorridas según el encoder: BA=%f mm, BB=%f mm LA=%f mm LB=%f mm\n", d_pulsos_BA,d_pulsos_BB,d_pulsos_LA,d_pulsos_LB);
	}
}


int main(){
	printf("iniciando herramientas \n");
	wiringPiSetup();
	
	// Configuración de pines de microcontroladores
	pinMode(EN,OUTPUT);

	pinMode(DIR_BA,OUTPUT);
	pinMode(STEP_BA,OUTPUT);

	pinMode(DIR_BB,OUTPUT);
	pinMode(STEP_BB,OUTPUT);

	pinMode(DIR_LA,OUTPUT);
	pinMode(STEP_LA,OUTPUT);

	pinMode(DIR_LB,OUTPUT);
	pinMode(STEP_LB,OUTPUT);

	// Configuración de entradas y salidas del multiplexor
	pinMode(SALIDA_INT,INPUT);
	pinMode(A_SELECT_INT,OUTPUT);
	pinMode(B_SELECT_INT,OUTPUT);
	pinMode(C_SELECT_INT,OUTPUT);
	pinMode(D_SELECT_INT,OUTPUT);
	
	// Configuración de pines encoders
    pinMode(PIN_ENCODER_BA, INPUT);
	pinMode(PIN_ENCODER_BB, INPUT);
	pinMode(PIN_ENCODER_LA, INPUT);
	pinMode(PIN_ENCODER_LB, INPUT);

	//Todo a cero por si acaso
	digitalWrite (DIR_BA, HIGH);
	digitalWrite (STEP_BA, HIGH);
	digitalWrite (DIR_BB, HIGH);
	digitalWrite (STEP_BB, HIGH);
	digitalWrite (DIR_LA, HIGH);
	digitalWrite (STEP_LA, HIGH);
	digitalWrite (DIR_LB, HIGH);
	digitalWrite (STEP_LB, HIGH);
	digitalWrite (EN, HIGH);

	lectura_posicion();
	// Creacción hilos
	piThreadCreate(Lectura_interruptores);
	piThreadCreate(Motor_BA);
	piThreadCreate(Motor_BB);
	piThreadCreate(Motor_LA);
	piThreadCreate(Motor_LB);
	piThreadCreate(Guardado_Posicion);
	
	int modo = 0;
	int n_fases = 0;
	num_mot = 0;
	sgn_BA = 0;
	sgn_BB = 0;
	sgn_LA = 0;
	sgn_LB = 0;
	
	reset =0;
	printf("\nHa elegido el subprograma del experimento.\n");
	int final=0;
	
	while(modo != 4){
		printf("Escoja la función a realizar:\n");
		printf("1. Conocer posición de los motores.\n");
		printf("2. Iniciar el experimento.\n");
		printf("3. Parar motores.\n");
		scanf("%i",&modo);
		
		if(modo == 1){
			lectura_posicion();
			printf("Actualmente los motores se encuentran en:\n");
			printf("BASE A (Motor 1): %i mm, BASE B (Motor 2): %i mm, PARED A (Motores 3 y 4): %i mm, PARED B (Motores 5 y 6): %i mm\n", pos_BA, pos_BB, pos_LA, pos_LB);
			num_mot = 0;	
		}
		
		if(modo == 2){
			problema=0;
			final=0;
			lectura_posicion();

			printf("Actualmente los motores se encuentran en:\n");
			printf("BASE A (Motor 1): %i mm, BASE B (Motor 2): %i mm, PARED A (Motores 3 y 4): %i mm, PARED B (Motores 5 y 6): %i mm\n", pos_BA, pos_BB, pos_LA, pos_LB);
			printf("\n");

			//-------------------------------------------------------------
			//Leemos el archivo csv
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
			int tasas_BA[MAX_FILAS], distancia_BA[MAX_FILAS], tasas_BB[MAX_FILAS], distancia_BB[MAX_FILAS];
			int tasas_LA[MAX_FILAS], distancia_LA[MAX_FILAS], tasas_LB[MAX_FILAS], distancia_LB[MAX_FILAS];

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
							break;
						case 1:
							tasas_BA[fila - 1] = atof(token);
							break;
						case 2:
							distancia_BB[fila - 1] = atof(token);
							break;
						case 3:
							tasas_BB[fila - 1] = atof(token);
							break;
						case 4:
							distancia_LA[fila - 1] = atof(token);
							break;
						case 5:
							tasas_LA[fila - 1] = atof(token);
							break;
						case 6:
							distancia_LB[fila - 1] = atof(token);
							break;
						case 7:
							tasas_LB[fila - 1] = atof(token);
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
			
			//-------------------------------------------------------------
			// Imprimir los datos para verificar que se hayan leído correctamente.
			int suma_BA=pos_BA;
			int suma_BB=pos_BB;
			int suma_LA=pos_LA;
			int suma_LB=pos_LB;
			for (int i = 0; i < fila - 1; i++) {
				printf("%d, %d, %d, %d, %d, %d, %d, %d\n", 
					distancia_BA[i], tasas_BA[i], distancia_BB[i], tasas_BB[i],
					distancia_LA[i], tasas_LA[i], distancia_LB[i], tasas_LB[i]);
				suma_BA=suma_BA+distancia_BA[i];
				suma_BB=suma_BB+distancia_BB[i];
				suma_LA=suma_LA+distancia_LA[i];
				suma_LB=+suma_LB+distancia_LB[i];
				if ((distancia_BA[i]!=0)&&(tasas_BA[i]<1)){
					printf("ERROR FATAL. LOS VALORES DE TASAS EN EL CSV NO SON VÁLIDOS.");
					exit(1);
				}
				if ((distancia_BB[i]!=0)&&(tasas_BB[i]<1)){
					printf("ERROR FATAL. LOS VALORES DE TASAS EN EL CSV NO SON VÁLIDOS.");
					exit(1);
				}
				if ((distancia_LA[i]!=0)&&(tasas_LA[i]<1)){
					printf("ERROR FATAL. LOS VALORES DE TASAS EN EL CSV NO SON VÁLIDOS.");
					exit(1);
				}
				if ((distancia_LB[i]!=0)&&(tasas_LB[i]<1)){
					printf("ERROR FATAL. LOS VALORES DE TASAS EN EL CSV NO SON VÁLIDOS.");
					exit(1);
				}
				if (((tasas_BA[i]<0)||(tasas_BA[i]>20000)||(tasas_BB[i]<0)||(tasas_BB[i]>20000))||((tasas_LA[i]<0)||(tasas_LA[i]>20000)||(tasas_LB[i]<0)||(tasas_LB[i]>20000))){
					printf("ERROR FATAL. LOS VALORES DE TASAS EN EL CSV NO SON VÁLIDOS.");
					exit(1);
				}
			}
			printf("suma_BA=%d\n", suma_BA);
			printf("suma_BB=%d\n", suma_BB);
			printf("suma_LA=%d\n", suma_LA);
			printf("suma_LB=%d\n", suma_LB);

			if ((suma_BA < 0) || (suma_BA>420)){
				printf("ERROR FATAL. LOS VALORES DE DISTANCIAS (BA) EN EL CSV NO SON VÁLIDOS.");
				exit(1);
			}
			if ((suma_BB < 0) || (suma_BB>420)) {
				printf("ERROR FATAL. LOS VALORES DE DISTANCIAS (BB) EN EL CSV NO SON VÁLIDOS.");
				exit(1);
			}
			if ((suma_LA < 0) || (suma_LA>200)){
				printf("ERROR FATAL. LOS VALORES DE DISTANCIAS (LA) EN EL CSV NO SON VÁLIDOS.");
				exit(1);
			}
			if ((suma_LB < 0) || (suma_LB>200)) {
				printf("ERROR FATAL. LOS VALORES DE DISTANCIAS (LB) EN EL CSV NO SON VÁLIDOS.");
				exit(1);
			}

			////-------------------------------------------------------------
			n_fases=fila-1;
			printf("Este experimento tendrá %d fases\n", n_fases);
			
			printf("Puede proceder a colocar el material en el interior.\n");
			printf("Introduzca un 1 cuando haya acabado.");
			int seguridad = 0;
			while (seguridad != 1) {
			scanf("%i", &seguridad);
			}

			int momento;
			//Realizamos las distintas fases de movimiento
			if(final==0){
				for (int i = 0; i < fila - 1; i++) {
					if (problema==0){
						momento=i+1;
						printf("Fase %d del movimiento\n", momento);
						d_BA=distancia_BA[i];
						d_BB=distancia_BB[i];
						d_LA=distancia_LA[i];
						d_LB=distancia_LB[i];
						tasa_BA=tasas_BA[i];
						tasa_BB=tasas_BB[i];
						tasa_LA=tasas_LA[i];
						tasa_LB=tasas_LB[i];
						movimiento();
						if (momento==fila-1){
							final=1;
						}
					}
				}
			}
			printf("Experimento finalizado.\n");
			num_mot = 0;
		}

		if(modo == 3){
			digitalWrite (DIR_BA, HIGH);
			digitalWrite (STEP_BA, HIGH);
			
			digitalWrite (DIR_BB, HIGH);
			digitalWrite (STEP_BB, HIGH);

			digitalWrite (DIR_LA, HIGH);
			digitalWrite (STEP_LA, HIGH);
			
			digitalWrite (DIR_LB, HIGH);
			digitalWrite (STEP_LB, HIGH);

			delay(200);
			digitalWrite(EN, HIGH);
		}
	}

	printf("WARNING: MOTOR EN EXTREMO DEL RECORRIDO");

	digitalWrite (DIR_BA, HIGH);
	digitalWrite (STEP_BA, HIGH);
	
	digitalWrite (DIR_BB, HIGH);
	digitalWrite (STEP_BB, HIGH);

	digitalWrite (DIR_LA, HIGH);
	digitalWrite (STEP_LA, HIGH);
	
	digitalWrite (DIR_LB, HIGH);
	digitalWrite (STEP_LB, HIGH);

	delay(200);
	digitalWrite(EN, HIGH);

	tdelay_BA = 1000*1000/(2*500);
	tdelay_BB = 1000*1000/(2*500);
	tdelay_LA = 1000*1000/(2*500);
	tdelay_LB = 1000*1000/(2*500);

	int pos_motor;

	if((int_BA_nm==1)||(int_BA_m==1)){
		if(int_BA_m==1){
			digitalWrite(DIR_BA,HIGH);
			pos_motor=0;
			delay(20);
		}
		else if(int_BA_nm==1){
			digitalWrite(DIR_BA,LOW);
			pos_motor=1;
			delay(20);
		}
		reset=1;
		n_pasos_BA = 10000;

		while((int_BA_nm==1)||(int_BA_m==1)){
			i_BA = 0;
			printf("BASE A.\n");
			printf("interruptor motor=%i, interruptor no motor=%i\n", int_BA_m, int_BA_nm);
		}

		reset=0;
		n_pasos_BA = -1;
			
		//Por si acaso
		digitalWrite(STEP_BA, HIGH);
		delay(20);
			
		//Todo a cero por si acaso
		digitalWrite(DIR_BA, HIGH);
		digitalWrite(EN, HIGH);
			
		if(pos_motor==0){
			pos_BA = 0;
		}
		else if(pos_motor==1){
			pos_BA = 200;
		}
	}


	if((int_BB_nm==1)||(int_BB_m==1)){
		if(int_BB_m==1){
			digitalWrite(DIR_BB,HIGH);
			pos_motor=0;
			delay(20);
		}
		else if(int_BB_nm==1){
			digitalWrite(DIR_BB,LOW);
			pos_motor=1;
			delay(20);
		}
		reset=1;
		n_pasos_BB = 10000;

		while((int_BB_nm==1)||(int_BB_m==1)){
			i_BB = 0;
			printf("BASE B.\n");
			printf("interruptor motor=%i, interruptor no motor=%i\n", int_BB_m, int_BB_nm);
		}

		reset=0;
		n_pasos_BB = -1;
			
		//Por si acaso
		digitalWrite(STEP_BB, HIGH);
		delay(20);
			
		//Todo a cero por si acaso
		digitalWrite(DIR_BB, HIGH);
		digitalWrite(EN, HIGH);
			
		if(pos_motor==0){
			pos_BB = 0;
		}
		else if(pos_motor==1){
			pos_BB = 200;
		}
	}


	if((int_LA_nm==1)||(int_LA_m==1)){
		if(int_LA_m==1){
			digitalWrite(DIR_LA,LOW);
			pos_motor=0;
			delay(20);
		}
		else if(int_LA_nm==1){
			digitalWrite(DIR_LA,HIGH);
			pos_motor=1;
			delay(20);
		}
		reset=1;
		n_pasos_LA = 10000;

		while((int_LA_nm==1)||(int_LA_m==1)){
			i_LA = 0;
			printf("LADO B.\n");
			printf("interruptor motor=%i, interruptor no motor=%i\n", int_LA_m, int_LA_nm);
		}

		reset=0;
		n_pasos_LA = -1;
			
		//Por si acaso
		digitalWrite(STEP_LA, HIGH);
		delay(20);
			
		//Todo a cero por si acaso
		digitalWrite(DIR_LA, HIGH);
		digitalWrite(EN, HIGH);
			
		if(pos_motor==0){
			pos_LA = 0;
		}
		else if(pos_motor==1){
			pos_LA = 200;
		}
	}


	if((int_LB_nm==1)||(int_LB_m==1)){
		if(int_LB_m==1){
			digitalWrite(DIR_LB,HIGH);
			pos_motor=0;
			delay(20);
		}
		else if(int_LB_nm==1){
			digitalWrite(DIR_LB,LOW);
			pos_motor=1;
			delay(20);
		}
		reset=1;
		n_pasos_LB = 10000;

		while((int_LB_nm==1)||(int_LB_m==1)){
			i_LB = 0;
			printf("LADO B.\n");
			printf("interruptor motor=%i, interruptor no motor=%i\n", int_LB_m, int_LB_nm);
		}

		reset=0;
		n_pasos_LB = -1;
			
		//Por si acaso
		digitalWrite(STEP_LB, HIGH);
		delay(20);
			
		//Todo a cero por si acaso
		digitalWrite(DIR_LB, HIGH);
		digitalWrite(EN, HIGH);
			
		if(pos_motor==0){
			pos_LB = 0;
		}
		else if(pos_motor==1){
			pos_LB = 200;
		}
	}

	printf("FINAL DEL PROGRAMA.\n");
	exit(1);

	return 0;
}
