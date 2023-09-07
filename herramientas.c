#include <wiringPi.h>
#include <pthread.h>
#include <math.h>
#include <stdio.h>
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

int int_BA_m, int_BA_nm, int_BB_m, int_BB_nm, int_LA_m, int_LA_nm, int_LB_m, int_LB_nm;
int i_BA, i_BB, i_LA, i_LB, n_pasos_BA, n_pasos_BB, n_pasos_LA, n_pasos_LB;
int tdelay_BA, tdelay_BB, tdelay_LA, tdelay_LB, sgn_BA, sgn_BB, sgn_LA, sgn_LB, count_BA, count_BB, count_LA, count_LB;
int num_mot;

int reset;

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


//Función que lleva los motores a la posición 0
void reset_pos(){
	printf("Comienzo del reseteo.\n");
	sgn_BA = -1;
	sgn_BB = -1;
	sgn_LA = -1;
	sgn_LB = -1;
	
	tdelay_BA = 1000*1000/(2*f0);
	tdelay_BB = 1000*1000/(2*f0);
	tdelay_LA = 1000*1000/(2*f0);
	tdelay_LB = 1000*1000/(2*f0);
	
	//Todo a cero por si acaso
	digitalWrite(STEP_BA, HIGH);
	digitalWrite(DIR_BA, HIGH);
	
	digitalWrite(STEP_BB, HIGH);
	digitalWrite(DIR_BB, HIGH);

	digitalWrite(STEP_LA, HIGH);
	digitalWrite(DIR_LA, HIGH);
	
	digitalWrite(STEP_LB, HIGH);
	digitalWrite(DIR_LB, HIGH);

	digitalWrite(EN, HIGH);
	
	delay(200);
	printf("Todas las señales puestas a 0.\n");

	while((num_mot!=1&&num_mot!=2)&&(num_mot!=3&&num_mot!=4)){
		printf("¿Qué motor desea mover?\n");
		printf("1. Motor 1 (BASE A)\n");
		printf("2. Motor 2 (BASE B)\n");
		printf("3. Motores 3 y 4 (PARED A)\n");
		printf("4. Motores 5 y 6 (PARED B)\n");
		scanf("%i",&num_mot);
	}

	if(num_mot==1){
		printf("Comienzo reseteo Base A.\n");
		digitalWrite(EN, LOW);
	    delay(200);
	    digitalWrite(DIR_BA, LOW); //Hacia el motor
		delay(20);
		printf("Motores encendidos.\n");
        
	    n_pasos_BA = 10000;

        while(int_BA_m==0){
			i_BA = 0;
			printf("Esperando a llegar a la posición de reseteo.\n");
			printf("interruptores= %i\n", int_BA_m);
	    }

		//Por si acaso
		n_pasos_BA = -1;
		digitalWrite(STEP_BA, HIGH);
		delay(20);
		
		tdelay_BA = 1000*1000/(2*500);
		//Nos movemos en la dirección contraria hasta dejar de pulsar el interruptor
		digitalWrite(DIR_BA,HIGH);
		delay(20);
		
		reset=1;
		n_pasos_BA = 10000;

		while(int_BA_m==1){
			i_BA = 0;
			printf("Esperando a llegar a la posición de reseteo 2.\n");
			printf("interruptores= %i\n", int_BA_m);
		}

		reset=0;
		n_pasos_BA = -1;
		
		//Por si acaso
		digitalWrite(STEP_BA, HIGH);
		delay(20);
		
		//Todo a cero por si acaso
		digitalWrite(DIR_BA, HIGH);

		digitalWrite(EN, HIGH);
		printf("Final reseteo Base A.\n");
		pos_BA = 0;
	}

	else if(num_mot==2){
		printf("Comienzo reseteo Base B.\n");
		digitalWrite(EN, LOW);
	    delay(200);
	    digitalWrite(DIR_BB, LOW); //Hacia el motor
		delay(20);
		printf("Motores encendidos.\n");
        
	    n_pasos_BB = 10000;

        while(int_BB_m==0){
			i_BB = 0;
			printf("Esperando a llegar a la posición de reseteo.\n");
			printf("interruptores= %i\n", int_BB_m);
	    }

		//Por si acaso
		n_pasos_BB = -1;
		digitalWrite(STEP_BB, HIGH);
		delay(20);
		
		tdelay_BB = 1000*1000/(2*500);
		//Nos movemos en la dirección contraria hasta dejar de pulsar el interruptor
		digitalWrite(DIR_BB,HIGH);
		delay(20);
		
		reset=1;
		n_pasos_BB = 10000;

		while(int_BB_m==1){
			i_BB = 0;
			printf("Esperando a llegar a la posición de reseteo 2.\n");
			printf("interruptores= %i\n", int_BB_m);
		}

		reset=0;
		n_pasos_BB = -1;
		
		//Por si acaso
		digitalWrite(STEP_BB, HIGH);
		delay(20);
		
		//Todo a cero por si acaso
		digitalWrite(DIR_BB, HIGH);

		digitalWrite(EN, HIGH);
		printf("Final reseteo Base B.\n");
		pos_BB = 0;
	}

	else if(num_mot==3){
		printf("Comienzo reseteo Lado A.\n");
		digitalWrite(EN, LOW);
	    delay(200);
	    digitalWrite(DIR_LA, HIGH); //Hacia el motor
		delay(20);
		printf("Motores encendidos.\n");
        
	    n_pasos_LA = 10000;

        while(int_LA_m==0){
			i_LA = 0;
			printf("Esperando a llegar a la posición de reseteo.\n");
			printf("interruptores= %i\n", int_LA_m);
	    }

		//Por si acaso
		n_pasos_LA = -1;
		digitalWrite(STEP_LA, HIGH); //hacia el motor
		delay(20);
		
		tdelay_LA = 1000*1000/(2*500);
		//Nos movemos en la dirección contraria hasta dejar de pulsar el interruptor
		digitalWrite(DIR_LA, LOW);
		delay(20);
		
		reset=1;
		n_pasos_LA = 10000;

		while(int_LA_m==1){
			i_LA = 0;
			printf("Esperando a llegar a la posición de reseteo 2.\n");
			printf("interruptores= %i\n", int_LA_m);
		}

		reset=0;
		n_pasos_LA = -1;
		
		//Por si acaso
		digitalWrite(STEP_LA, HIGH);
		delay(20);
		
		//Todo a cero por si acaso
		digitalWrite(DIR_LA, HIGH);

		digitalWrite(EN, HIGH);
		printf("Final reseteo Lado A.\n");
		pos_LA = 0;
	}

	else if(num_mot==4){
		printf("Comienzo reseteo Lado B.\n");
		digitalWrite(EN, LOW);
	    delay(200);
	    digitalWrite(DIR_LB, LOW); //Hacia el motor
		delay(20);
		printf("Motores encendidos.\n");
        
	    n_pasos_LB = 10000;

        while(int_LB_m==0){
			i_LB = 0;
			printf("Esperando a llegar a la posición de reseteo.\n");
			printf("interruptores= %i\n", int_LB_m);
	    }

		//Por si acaso
		n_pasos_LB = -1;
		digitalWrite(STEP_LB, HIGH);
		delay(20);
		
		tdelay_LB = 1000*1000/(2*500);
		//Nos movemos en la dirección contraria hasta dejar de pulsar el interruptor
		digitalWrite(DIR_LB, HIGH);
		delay(20);
		
		reset=1;
		n_pasos_LB = 10000;

		while(int_LB_m==1){
			i_LB = 0;
			printf("Esperando a llegar a la posición de reseteo 2.\n");
			printf("interruptores= %i\n", int_LB_m);
		}

		reset=0;
		n_pasos_LB = -1;
		
		//Por si acaso
		digitalWrite(STEP_LB, HIGH);
		delay(20);
		
		//Todo a cero por si acaso
		digitalWrite(DIR_LB, HIGH);

		digitalWrite(EN, HIGH);
		printf("Final reseteo Lado B.\n");
		pos_LA = 0;
	}
	printf("Final de la función de reseteo.\n");
}


//Función que almacena el valor de la posición en data y luego lee la posición
void lectura_posicion(){
	char data[16];
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
	
	sscanf(data, "%d,%d,%d,%d\n", &pos_BA, &pos_BB, &pos_LA, &pos_LB); //Asignamos valores a variables

	if ((pos_BA < 0) || (pos_BA>200)){
		printf("ERROR FATAL. POR FAVOR, REALICE UN RESETEO.");
	}
	if ((pos_BB < 0) || (pos_BB>200)) {
		printf("ERROR FATAL. POR FAVOR, REALICE UN RESETEO.");
	}
	if ((pos_LA < 0) || (pos_LA>200)){
		printf("ERROR FATAL. POR FAVOR, REALICE UN RESETEO.");
	}
	if ((pos_LB < 0) || (pos_LB>200)) {
		printf("ERROR FATAL. POR FAVOR, REALICE UN RESETEO.");
	}
}


//Función que permite desplazar el motor una distancia concreta
void desplazamiento(){
	void piLock();
	pulso_BA = 0;
	pulso_BB = 0;
	pulso_LA = 0;
	pulso_LB = 0;
	void piUnLock();
	
	int d=-1000;
	float d_pulsos=0;
	
	tdelay_BA = 1000*1000/(2*f0);
	tdelay_BB = 1000*1000/(2*f0);
	tdelay_LA = 1000*1000/(2*f0);
	tdelay_LB = 1000*1000/(2*f0);
	
	//Elección de los motores a mover
	while((num_mot!=1&&num_mot!=2)&&(num_mot!=3&&num_mot!=4)){
		printf("¿Qué motor desea mover?\n");
		printf("1. Motor 1 (BASE A)\n");
		printf("2. Motor 2 (BASE B)\n");
		printf("3. Motores 3 y 4 (PARED A)\n");
		printf("4. Motores 5 y 6 (PARED B)\n");
		scanf("%i",&num_mot);
	}

	if(num_mot==1){
		while(((pos_BA+d)<0)||((pos_BA+d)>420)){
			printf("¿Qué distancia desea desplazar? \n Introduzca un valor para que el motor se encuentre entre 0-420 mm. Signo positivo movimiento hacia 420, signo negativo hacia 0.\n");
			scanf("%i",&d);
		}
		//Habilito el motor 1
		digitalWrite(EN, LOW);
		printf("Enable on \n");
		delay(300);
		
		//Miramos dirección
		if(d<0){ //Negativa es hacia motor
			digitalWrite(DIR_BA, LOW);
			delay(200);
			sgn_BA = -1;
		}
		if(d>0){ //Positiva es hacia no motor
			digitalWrite(STEP_BA, HIGH);
			delay(200);
			sgn_BA = 1;
		}
		delay(200);
		n_pasos_BA = fabs(d)*k/2; //cada vuelta es 2 mm
		
		while(i_BA<n_pasos_BA){
			delay(20);
		}
		n_pasos_BA=-1;
		d_pulsos=pulso_BA*2/500; //DISTANCIA RECORRIDA SEGUN PULSOS DEL ENCODER
	}

	if(num_mot==2){
		while(((pos_BB+d)<0)||((pos_BB+d)>400)){
			printf("¿Qué distancia desea desplazar? \n Introduzca un valor para que el motor se encuentre entre 0-400 mm. Signo positivo movimiento hacia 420, signo negativo hacia 0.\n");
			scanf("%i",&d);
		}

		//Habilito el motor 2
		digitalWrite(EN, LOW);
		delay(200);
		
		//Miramos dirección
		if(d<0){ //Negativa es hacia motor
			digitalWrite(DIR_BB, LOW);
			sgn_BB = -1;
		}
		 if(d>0){ //Positiva es hacia no motor
			digitalWrite(DIR_BB, HIGH);
			sgn_BB = 1;
		}
		
		delay(200);
		
		n_pasos_BB = fabs(d)*k/2;

		while(i_BB<n_pasos_BB){
			delay(20);
		}
		n_pasos_BB=-1;
		d_pulsos=pulso_BB*2/500; //DISTANCIA RECORRIDA SEGUN PULSOS DEL ENCODER
	}

	if(num_mot==3){
		printf("Posicion=%d\n", pos_LA);
		printf("Distancia=%d\n", d);
		while(((pos_LA+d))<0||((pos_LA+d))>200){
			printf("¿Qué distancia desea desplazar? \n Introduzca un valor para que el motor se encuentre entre 0-200 mm. Signo positivo movimiento hacia 200, signo negativo hacia 0.\n");
			scanf("%i",&d);
			printf("Distancia=%d\n", d);
		}
		//Habilito el motor 1
		digitalWrite(EN, LOW);
		printf("Enable on \n");
		delay(200);
		
		//Miramos dirección
		if(d<0){ //Negativa es hacia motor (extension)
			digitalWrite(DIR_LA, HIGH);
			sgn_LA = -1;
		}
		 if(d>0){ //Positiva es hacia el no motor (compresion)
			digitalWrite(DIR_LA, LOW);
			sgn_LA = 1;
		}
		delay(200);
		
		n_pasos_LA = fabs(d)*k/2;

		while(i_LA<n_pasos_LA){
			delay(20);
		}
		n_pasos_LA=-1;
		d_pulsos=pulso_LA*2/500; //DISTANCIA RECORRIDA SEGUN PULSOS DEL ENCODER
	}

	if(num_mot==4){
		while(((pos_LB+d)<0)||((pos_LB+d)>200)){
			printf("¿Qué distancia desea desplazar? \n Introduzca un valor para que el motor se encuentre entre 0-200 mm. Signo positivo movimiento hacia 200, signo negativo hacia 0.\n");
			scanf("%i",&d);
		}

		//Habilito el motor 2
		digitalWrite(EN, LOW);
		delay(200);
		
		//Miramos dirección
		if(d<0){ //Negativa es hacia motor
			digitalWrite(DIR_LB, LOW);
			sgn_LB = -1;
		}
		 if(d>0){ //Positiva es hacia motor
			digitalWrite(DIR_LB, HIGH);
			sgn_LB = 1;
		}
		
		delay(200);
		
		n_pasos_LB = fabs(d)*k/2;

		while(i_LB<n_pasos_LB){
			delay(20);
		}
		n_pasos_LB=-1;
		d_pulsos=pulso_LB*2/500; //DISTANCIA RECORRIDA SEGUN PULSOS DEL ENCODER
	}

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

	// Comprobamos con el encoder si se mueve lo que configuramos
	float error_relativo=0;
	error_relativo=fabs(fabs(d)-d_pulsos)*100/fabs(d);
	if ((error_relativo>5)&&(num_mot!=4)){
		printf("WARNING: Error relativo de distancias medidas con encoder mayor del 10 porciento\n");
	}
	if (num_mot!=4){
		printf ("Distancia recorrida según el encoder=%f mm\n", d_pulsos);
	}
	printf ("Valores BA=%d, BB=%d, LA=%d, LB=%d,  mm\n", pulso_BA,pulso_BB,pulso_LA,pulso_LB);
}


void posicionamiento(){
	void piLock();
	pulso_BA = 0;
	pulso_BB = 0;
	pulso_LA = 0;
	pulso_LB = 0;
	void piUnLock();
	
	int pos = -1;
	float d_pulsos=0;
	int d;
	
	tdelay_BA = 1000*1000/(2*f0);
	tdelay_BB = 1000*1000/(2*f0);
	tdelay_LA = 1000*1000/(2*f0);
	tdelay_LB = 1000*1000/(2*f0);
	
	while((num_mot!=1&&num_mot!=2)&&(num_mot!=3&&num_mot!=4)){
		printf("¿Qué motor desea mover?\n");
		printf("1. Motor 1 (BASE A)\n");
		printf("2. Motor 2 (BASE B)\n");
		printf("3. Motores 3 y 4 (PARED A)\n");
		printf("4. Motores 5 y 6 (PARED B)\n");
		scanf("%i",&num_mot);
	}
	
	if(num_mot==1){
		while((pos<0)||(pos>400)){ //se mueve el de la base A
			printf("¿A qué posición desea mover el motor 1? (0-420) mm\n");
			scanf("%i",&pos);
		}
		
		d = pos-pos_BA; //calculo la distancia a mover
		
		//Habilito el motor 1
		digitalWrite(EN, LOW);
		delay(200);
		
		//Miramos dirección
		if(d<0){ //Negativa es hacia motor
			digitalWrite(DIR_BA, LOW);
			sgn_BA = -1;
		}
		 if(d>0){ //Positiva es hacia no motor
			digitalWrite(DIR_BA, HIGH);
			sgn_BA = 1;
		}
		
		delay(200);
		n_pasos_BA = fabs(d)*k/2;

		while(i_BA<n_pasos_BA){
			delay(20);
		}
		n_pasos_BA=-1;
		d_pulsos=pulso_BA*2/500; //DISTANCIA RECORRIDA SEGUN PULSOS DEL ENCODER
	}
	
	if(num_mot==2){ //se mueve el motor de la base B
		while((pos<0)||(pos>400)){
			printf("¿A qué posición desea mover el motor 2? (0-420) mm\n");
			scanf("%i",&pos);
		}
		
		d = pos-pos_BB;
		
		//Habilito el motor 2
		digitalWrite(EN, LOW);
		delay(200);
		
		//Miramos dirección
		if(d<0){ //Negativa es hacia motor
			digitalWrite(DIR_BB, LOW);
			sgn_BB = -1;
		}
		if(d>0){ //Positiva es hacia no motor
			digitalWrite(DIR_BB, HIGH);
			sgn_BB =1;
		}
		delay(200);
		n_pasos_BB = fabs(d)*k/2;
		
		while(i_BB<n_pasos_BB){
			delay(20);
		}

		n_pasos_BB = -1;
		d_pulsos=pulso_BB*2/500; //DISTANCIA RECORRIDA SEGUN PULSOS DEL ENCODER
	}
	
	if(num_mot==3){
		while((pos<0)||(pos>200)){ //se mueve el motor del lado A
			printf("¿A qué posición desea mover el motor 1? (0-200) mm\n");
			scanf("%i",&pos);
		}
		
		d = pos-pos_LA; //calculo la distancia a mover
		
		//Habilito los motores 3 y 4
		digitalWrite(EN, LOW);
		delay(200);
		
		//Miramos dirección
		if(d<0){ //Negativa es hacia motor (extension)
			digitalWrite(DIR_LA, HIGH);
			sgn_LA = -1;
		}
		 if(d>0){ //Positiva es hacia no motor (compresion)
			digitalWrite(DIR_LA, LOW);
			sgn_LA = 1;
		}
		
		delay(200);
		n_pasos_LA = fabs(d)*k/2;

		while(i_LA<n_pasos_LA){
			delay(20);
		}

		n_pasos_LA=-1;
		d_pulsos=pulso_LA*2/500; //DISTANCIA RECORRIDA SEGUN PULSOS DEL ENCODER
	}
	
	if(num_mot==4){ //se mueve el motor del lado B
		while((pos<0)||(pos>200)){
			printf("¿A qué posición desea mover el motor 2? (0-200) mm\n");
			scanf("%i",&pos);
		}
		
		d = pos-pos_LB;
		
		//Habilito los motores 5 y 6
		digitalWrite(EN, LOW);
		delay(200);
		
		//Miramos dirección
		if(d<0){ //Negativa es hacia motor (extension)
			digitalWrite(DIR_LB, LOW);
			sgn_LB = -1;
		}
		if(d>0){ //Positiva es hacia no motor (compresion)
			digitalWrite(DIR_LB, HIGH);
			sgn_LB =1;
		}

		delay(200);
		n_pasos_LB = fabs(d)*k/2;
		
		while(i_LB<n_pasos_LB){
			delay(20);
		}

		n_pasos_LB = -1;
		d_pulsos=pulso_LB*2/500; //DISTANCIA RECORRIDA SEGUN PULSOS DEL ENCODER
	}
	

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

	// Comprobamos con el encoder si se mueve lo que configuramos
	float error_relativo=0;
	error_relativo=fabs(fabs(d)-d_pulsos)*100/fabs(d);
	if ((error_relativo>5)&&(num_mot!=4)){
		printf("WARNING: Error relativo de distancias medidas con encoder mayor del 10 porciento\n");
	}
	if (num_mot!=4){
		printf ("Distancia recorrida según el encoder=%f mm\n", d_pulsos);
	}
}


int main(){
	printf("Iniciando el submenú de herramientas\n");
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
	
	// Todo a cero por si acaso
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
	piThreadCreate(Medir_encoder_BA);
	piThreadCreate(Medir_encoder_BB);
	piThreadCreate(Medir_encoder_LA);
	piThreadCreate(Medir_encoder_LB);
	
	int modo = 0;
	num_mot = 0;
	sgn_BA = 0;
	sgn_BB = 0;
	sgn_LA = 0;
	sgn_LB = 0;
	
	reset =0;	
	printf("\nHa elegido el programa de herramientas.\n");
	
	while(modo != 6){
		printf("Escoja:\n");
		printf("1. Realizar el reseteo de la posición de un motor.\n");
		printf("2. Desplazar un motor una distancia.\n");
		printf("3. Colocar un motor en posición determinada.\n");
		printf("4. Conocer la posición de los motores.\n");
		printf("5. Parar motores.\n");
		printf("6. Volver al menú de control.\n");
		scanf("%i",&modo);
		
		if(modo == 1){
			reset_pos();
			num_mot = 0;
		}
		
		if(modo == 2){
			lectura_posicion();
			printf("Actualmente los motores se encuentran en:\n");
			printf("BASE A (Motor 1): %i mm, BASE B (Motor 2): %i mm, PARED A (Motores 3 y 4): %i mm, PARED B (Motores 5 y 6): %i mm\n", pos_BA, pos_BB, pos_LA, pos_LB);
			printf("\n");
			
			desplazamiento();
			num_mot = 0;
		}
		
		if(modo == 3){
			lectura_posicion();
			printf("Actualmente los motores se encuentran en:\n");
			printf("BASE A (Motor 1): %i mm, BASE B (Motor 2): %i mm, PARED A (Motores 3 y 4): %i mm, PARED B (Motores 5 y 6): %i mm\n", pos_BA, pos_BB, pos_LA, pos_LB);
			printf("\n");
			
			posicionamiento();
			num_mot = 0;
		}

		if(modo == 4){
			lectura_posicion();
			printf("Actualmente los motores se encuentran en:\n");
			printf("BASE A (Motor 1): %i mm, BASE B (Motor 2): %i mm, PARED A (Motores 3 y 4): %i mm, PARED B (Motores 5 y 6): %i mm\n", pos_BA, pos_BB, pos_LA, pos_LB);
			num_mot = 0;	
		}

		if(modo == 5){
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
	return 0;
}
