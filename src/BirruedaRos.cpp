#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <rclc/executor.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h> 
#include <ESP32Encoder.h>
#include <iostream>

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

#define LED_PIN 2  // Pin del LED en el ESP

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

const char* ssid = "UA-Alumnos";
const char* password = "41umn05WLC";
const char* agent_ip = "192.168.1.100";  // Dirección IP del agente micro-ROS
const uint16_t agent_port = 8888;        // Puerto del agente micro-ROS

void error_loop() {
  while(1) {
    delay(100);
  }
}

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg->data == 1) {
    digitalWrite(LED_PIN, HIGH);  // Enciende LED
  } else {
    digitalWrite(LED_PIN, LOW);   // Apaga LED
  }
}

// ROBO DEL CODIGO DE BIRRUEDA. PODEMOS HACER UN .h PARA IMPORTARLO

ESP32Encoder encoder1;
ESP32Encoder encoder2;
char variable ;

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

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Asegúrate de que el LED esté apagado inicialmente

  Serial.begin(115200);

  // Configurar la conexión Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conectado a la red WiFi");

  // Establecer el transporte a través de Wi-Fi
  //set_microros_wifi_transports(ssid, password, agent_ip, agent_port);

  allocator = rcl_get_default_allocator();

  // Crear opciones de inicialización
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Crear nodo
  RCCHECK(rclc_node_init_default(&node, "esp_subscriber_node", "", &support));

  // Crear suscripción
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  // Crear executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  // Agregar suscripción al executor
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  // Ejecutar el executor
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
