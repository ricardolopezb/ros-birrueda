//Incluimos las librerías
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h> 
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <iostream>

//defini las variables del encoder
ESP32Encoder encoder1;
ESP32Encoder encoder2;
char variable ;

const char* ssid = "UA-Alumnos"; //Nombre de la red WIFI
const char* password = "41umn05WLC"; //Contraseña de la red WIFI

WiFiServer  server(80);
String webpage = "";
String header = "";

// Definición de pines para el Motor 1
#define ENABLE1 5
#define IN11 25
#define IN12 26
#define ENC1A 12
#define ENC1B 14

// Definición de pines para el Motor 2
#define ENABLE2 18
#define IN21 16
#define IN22 17
#define ENC2A 15
#define ENC2B 27

//inicializo todas las varibles que se utilizarán a continuación
unsigned long tiempo_anterior = 0;
float velocidad_actual1 = 0;
float velocidad_actual2 = 0;
float velocidad_deseada1 = 0;
float velocidad_deseada2 = 0;
float velocidad_deseadax1 = 0;
float velocidad_deseadax2 = 0;
float velocidad_original1 = 0;
float velocidad_original2 = 0;
float Kp = 1.38;
float Ki = 0.03;
float error_acumulado1 = 0;
float error_acumulado2 = 0;
float valor=0;


//FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-
//FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-
//FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-
//FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-FUNCIONES-

void iniciarWIFI(){ //Función para conectar el ESP32 a la red WIFI

  Serial.print("Conectando a  ");
  Serial.println(ssid);
  //Conectamos el esp a la red wifi
  WiFi.begin(ssid, password);
  //Intentamos conectarnos a la red
  while (WiFi.status() != WL_CONNECTED) { //Imprimimos cuanodo ya estamos conectados
    delay(500);
    Serial.print(".");
  }
  //Si logramos conectarnos mostramos la ip a la que nos conectamos
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
 }

void setup() {

  // Inicia el monitor serie
  Serial.begin(9600);
  pinMode(IN11, OUTPUT); //configura pin como salida
  pinMode(IN12, OUTPUT); //configura pin como salida
  pinMode(ENABLE1, OUTPUT); //configura pin como salida
  digitalWrite(ENABLE1, HIGH); //establece el pin ENABLE1 en un estado alto
  encoder1.attachHalfQuad(ENC1A, ENC1B); //configurando el codificador para que funcione en modo de medio cuadrante
  encoder1.clearCount(); //restablece el contador del codificador 

  pinMode(IN21, OUTPUT); //configura pin como salida
  pinMode(IN22, OUTPUT); //configura pin como salida
  pinMode(ENABLE2, OUTPUT); //configura pin como salida
  digitalWrite(ENABLE2, HIGH); //establece el pin ENABLE2 en un estado alto
  encoder2.attachHalfQuad(ENC2A, ENC2B); //configurando el codificador para que funcione en modo de medio cuadrante
  encoder2.clearCount(); //restablece el contador del codificador 

  iniciarWIFI();//inicio el wifi llamando a la funion definida anteriormente
}

//FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - 
//FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - 
//FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - 
//FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - FUNCIONES MOVIMIENTO - 


void ADELANTEc(int pwm1, int pwm2) { //Funcion adelante para el cuadrado 
  analogWrite(IN11, pwm1); 
  analogWrite(IN12, 0);
  analogWrite(IN21, pwm2);
  analogWrite(IN22, 0);
}

void FRENARc() { //Funcion frenar para el cuadrado 
  analogWrite(IN11, 0);
  analogWrite(IN12, 0);
  analogWrite(IN21, 0);
  analogWrite(IN22, 0);
}

// Establecemos todas las funciones de movimiento funciones de movimiento, cada una envía una señal a ciertos pines analógicos para que se mueva
// o gire en la dirección pretenida según los vectores definicdos.
void ADELANTE(int pwm1, int pwm2) {

   if (pwm1 == pwm2)
  {
    for(int i = 0; i <= pwm1; i += 2.5) {
    analogWrite(IN11, i);
    analogWrite(IN12, 0);
    analogWrite(IN21, i);
    analogWrite(IN22, 0);
    delay(10);
  }
  }


  analogWrite(IN11, pwm1);
  analogWrite(IN12, 0);
  analogWrite(IN21, pwm2);
  analogWrite(IN22, 0);
}

void ATRAS(int pwm1, int pwm2) {
  
  if (pwm1 == pwm2)
  {
    for(int i = 0; i <= pwm1; i += 2.5) {
    analogWrite(IN11, 0);
    analogWrite(IN12, i);
    analogWrite(IN21, 0);
    analogWrite(IN22, i);
    delay(10);
  }
  }

  analogWrite(IN11, 0);
  analogWrite(IN12, pwm1);
  analogWrite(IN21, 0);
  analogWrite(IN22, pwm2);
}

void FRENAR() {
  analogWrite(IN11, 0);
  analogWrite(IN12, 0);
  analogWrite(IN21, 0);
  analogWrite(IN22, 0);
}

void calcularVelocidad() { //calculo de velocidad actual de la velocidad
  const int PULSOS_POR_REVOLUCION = 2400;
  unsigned long tiempo_actual = millis();
  unsigned long delta_tiempo = tiempo_actual - tiempo_anterior; //diferencias de tiempo

  if (delta_tiempo >= 1000) { //verifica si ha transcurrido al menos 1 segundo desde la última medición de velocidad. Si es así, se realiza el cálculo de la velocidad.
    long cuenta1 = encoder1.getCount();//cantidad de pulsos contados del enco 1
    velocidad_actual1 = (float)cuenta1 / PULSOS_POR_REVOLUCION * 60 / (delta_tiempo / 1000.0); //calcula la velocidad del motor 1 en rpm

    long cuenta2 = encoder2.getCount(); //cantidad de pulsos contados del enco 2
    velocidad_actual2 = (float)cuenta2 / PULSOS_POR_REVOLUCION * 60 / (delta_tiempo / 1000.0); //calcula la velocidad del motor 2 en rpm

    tiempo_anterior = tiempo_actual; //actualizo el rimpo anterior
    encoder1.clearCount(); //restablece contador de pulsos de los codificadores
    encoder2.clearCount(); //restablece contador de pulsos de los codificadores

    //los serial print los usamos para durante el desarrollo poder ver que llegue a velocidad objetivo el motor
    Serial.print("Velocidad actual Motor 1: ");
    Serial.print(velocidad_actual1);
    Serial.println(" RPM");

    Serial.print("Velocidad actual Motor 2: ");
    Serial.print(velocidad_actual2);
    Serial.println(" RPM");
  }
}

void controlPI() { //controlador proporcional-integral (PI) para dos motores
  float error1 = velocidad_deseada1 - velocidad_actual1; //eror para el motor 1 Velo deseada vs actual
  error_acumulado1 += error1; //acumula error en una variable
  if (error_acumulado1 > 255) error_acumulado1 = 255; //limita la acumulación de error
  else if (error_acumulado1 < -255) error_acumulado1 = -255;
  int ajuste1 = Kp * error1 + Ki * error_acumulado1; //calcula el valor de ajuste del motor 1
//hace lo mismo pero para el motor 2
  float error2 = velocidad_deseada2 - velocidad_actual2;
  error_acumulado2 += error2;
  if (error_acumulado2 > 255) error_acumulado2 = 255;
  else if (error_acumulado2 < -255) error_acumulado2 = -255;
  int ajuste2 = Kp * error2 + Ki * error_acumulado2;

  if (ajuste1 > 0 && ajuste2 > 0) {//motores deben moverse hacia adelante
    ADELANTE(ajuste1, ajuste2);
  } else if (ajuste1 < 0 &&  ajuste2 < 0) { //motores moverse hacia atras
    ATRAS(abs(ajuste1),abs(ajuste2));
  } 
  else { //frenar los motores
    FRENAR();
  }
}


void casos(String header2){
      Serial.println("ESTE ES EL HEADER BIENVENIDOS MUCHAS GRACIAS"); //Manera de identificar cual es el header, es decir la IP que recibe el ESP32
      Serial.println(header2);

    char str_valor = header2[5]; // Guarda en str_valor el caracter que recibe de la app.
    Serial.println(str_valor);
    String input = "";
    input=str_valor;

    Serial.print("Input recibido: ");
    Serial.println(input); // Imprime el dato recibido en el monitor serie para un mejor comprension
    
    // En función del INPUT recibido por la app, llamamos a la funcion de movimiento necesaria
    if (input=="v"){ //Llama a la funcion que corresponde ir hacia a guardal la veloicad objetivo
        String str_get = "GET /";
        String str_valor = "";
        int slashPosition = header2.indexOf('/');
        int spacePosition = header2.indexOf(' ', slashPosition);

        for (int i = slashPosition + 2; i <= spacePosition; i++) {
            str_valor = str_valor + header2.charAt(i);
        }
        // Convierte str_valor a un número de punto flotante y guárdalo en una variable única
         valor = str_valor.toFloat();
    }

      else if (input=="A") //Llama a la funcion que corresponde ir hacia adelante 
        {
        velocidad_deseadax1 = valor; //define el valor de la velocida en rpm
        velocidad_deseadax2 = valor;
        velocidad_deseada1 = map(velocidad_deseadax1, -60, 60, -255, 255); //mapea la velocidad rpm a pwm para luego hacer girar el motor
        velocidad_deseada2 = map(velocidad_deseadax2, -60, 60, -255, 255);
        calcularVelocidad();
        controlPI();
        
      }

      else if (input=="S") //Llama a la funcion que corresponde a ir hacia atras
      {
      velocidad_deseadax1 = -valor;
      velocidad_deseadax2 = -valor;
      velocidad_deseada1 = map(velocidad_deseadax1, -60, 60, -255, 255);
      velocidad_deseada2 = map(velocidad_deseadax2, -60, 60, -255, 255);
      calcularVelocidad();
      controlPI();      
      }  

      else if (input=="E") //Llama a la funcion que corresponde ir hacia adelante izq
      {
      velocidad_deseadax1 = valor;  // Guardamos las velocidades originales
      velocidad_deseadax2 = valor;

      velocidad_deseada2 = map(velocidad_deseadax2, -60, 60, -255, 255);
      velocidad_deseada1 = map(velocidad_deseadax1, -60, 60, -255, 255);
      velocidad_deseada1 *= 0.2;
      velocidad_deseada2 *= 0.8;
      calcularVelocidad();
      controlPI();
      } 

      else if (input=="T") //Llama a la funcion que corresponde ir hacia atras izq
      {
      velocidad_deseadax1 = -valor;  // Guardamos las velocidades originales
      velocidad_deseadax2 = -valor;

      velocidad_deseada2 = map(velocidad_deseadax2, -60, 60, -255, 255);
      velocidad_deseada1 = map(velocidad_deseadax1, -60, 60, -255, 255);
      velocidad_deseada1 *= 0.2;
      velocidad_deseada2 *= 0.8;
      calcularVelocidad();
      controlPI();
      } 

      else if (input=="D") //Llama a la funcion que corresponde ir hacia adelante derecha
      {
      velocidad_deseadax1 = valor;  // Guardamos las velocidades originales
      velocidad_deseadax2 = valor;

      velocidad_deseada2 = map(velocidad_deseadax2, -60, 60, -255, 255);
      velocidad_deseada1 = map(velocidad_deseadax1, -60, 60, -255, 255);
      velocidad_deseada1 *= 0.8;
      velocidad_deseada2 *= 0.2;
      calcularVelocidad();
      controlPI();
      } 

      else if (input=="R") //Llama a la funcion que corresponde a girar hacia atraz izq
      {
      velocidad_deseadax1 = -valor;  // Guardamos las velocidades originales
      velocidad_deseadax2 = -valor;

      velocidad_deseada2 = map(velocidad_deseadax2, -60, 60, -255, 255);
      velocidad_deseada1 = map(velocidad_deseadax1, -60, 60, -255, 255);
      velocidad_deseada1 *= 0.8;
      velocidad_deseada2 *= 0.2;
      calcularVelocidad();
      controlPI();
      } 

      else if (input=="Z"){ //Llama a la funcion que corresponde a realizar un cuadrado
      
        for (int j = 0; j < 4; j++) { // Repetir 4 veces para hacer un cuadrado
          // Avanzar
          for (int i = 0; i <= 170; i += 2.5) {
            ADELANTEc(i, i);
            delay(10);
          }
          ADELANTEc(170, 170);
          delay(4550);
          FRENARc();
          delay(500);

          // Girar (solo una rueda se mueve)
          for (int i = 0; i <= 170; i += 2.5) {
            ADELANTEc(i, 0);
            delay(10);
          }
          ADELANTEc(170, 0);
          delay(2140);
          FRENARc();
          delay(500);
        }

      }
      else if (input=="F") //Llama a la funcion que corresponde a frenar
      {
      velocidad_deseadax1 = 0;  // Guardamos las velocidades originales
      velocidad_deseadax2 = 0;

      velocidad_deseada2 = map(velocidad_deseadax2, -60, 60, -255, 255);
      velocidad_deseada1 = map(velocidad_deseadax1, -60, 60, -255, 255);
      FRENAR();
      calcularVelocidad();
      controlPI();
      } 
      else 
      {
        Serial.println("VALOR INCORRECTO"); //En caso de que input contenga un caracter no contemplado, devuelve el aviso de valor incorrecto.
      }

}


void parametrosWiFi(){ //Esta es la función principal, que se encarga de traer la IP desde la app y comunicarsela al VONT.

WiFiClient client = server.available(); //Se conecta mediante WiFi conn la app y se fija si está disponible para enviar o recibir ir información.

if (client){
  
    String currentline = ""; //Para guardar la info que llega del client

  while(client.connected()){ //Lleva a cabo las siguientesi instruccipones siempre y cuando la app se mantenga conectada al ESP#"32
    if (client.available()){  // Ejecuta en función de si la app está disponible para enviar o recibir info 
      char c = client.read(); //En caso de que esté disponible, lee la IP y la almacena en la variable c.
      Serial.write(c); //Escribe la IP leída para realizar seguimiento
      header += c ;      
      
      if (c == '\n'){ //Revisa si la cadena de la IP terminó.

        if(currentline.length() == 0 ){
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println("Connection: close");
          client.println();

          break; // Salir del ciclo while:
          
        }else { //si tiene una nueva línea, borra currentLine:
          currentline = "";
        }

      }else if (c != '\r'){ // Si se vuelva al inicio de la cadena, anexa c a currenline para no perderlo.
        currentline =+ c;
      }
      
    }
  }
  
  casos (header); //Con el valor de header corre la función casos para comunicarse y mover al VONT en función del carácter dado.
  header = ""; //Vvacía la variable header para poder volver a correr esta función
  client.stop(); //Y Corta comunicación con la app.
 }
 
}


void loop() {
    parametrosWiFi(); //Corre la función parametrosWiFi, que es la que se encarga de la comunicación y con la app y movimiento del VONT en loop.
}