import mqtt.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

MQTTClient mqtt;
//mqtt://10.0.0.100:1883
final String MQTT_BROKER = "mqtt://tom.uib.es";
final String MQTT_CLIENT_ID = "HMI_TSN9490";
final String TOPIC_SUBS = "TFG_jprodriguez/#";
final String INST_XCBR = "TFG_jprodriguez/SCADA/XCBR";
final String INST_TC_TVTR_st = "TFG_jprodriguez/SCADA/TC-TVTR/state";
final String INST_TC_TVTR_ty = "TFG_jprodriguez/SCADA/TC-TVTR/type";
final String ORD_XCBR = "TFG_jprodriguez/Maqueta/XCBR";
final String ORD_PIOC = "TFG_jprodriguez/Maqueta/PIOC";
final String ORD_PIOV = "TFG_jprodriguez/Maqueta/PIOV";
final String ORD_TC_TVTR = "TFG_jprodriguez/Maqueta/TC-TVTR";

final int YELLOW = #ffbd03;
final int GREEN = #33b249;
final int RED = #ED0800;
final int BLUE = #5783db;
final int ORANGE = #FF7600;
final int PURPLE = #a881af;
final int AZUL_MARINO = #1e213d;

boolean last_state_XCBR = false;
boolean current_state_XCBR = false;
boolean last_state_TC_TVTR = false;
boolean current_state_TC_TVTR = true;
int last_state_fault = 0;
int current_state_fault = 0;

float [] rx_TC_TVTR_values = new float[8];
boolean rx_XCBR_state = false;
boolean rx_PIOC_state = false;
boolean rx_PIOV_state = false;

boolean button_pressed = false;
boolean interruptor_pressed = false;

// Hacer matriz booleanos de cuadros botones
boolean [][] interac = {    // [when_mouse is on][when clicked]
  {false, false}, // Botón Open/Close XCBR
  {false, false}, // Botón Inicio/Parada Falla TC/TVTR
  {false, true}, // Botón Falla Total TC/TVTR
  {false, false}, // Botón Falla V TC/TVTR
  {false, false}, // Botón Falla C TC/TVTR
};

int [][] interac_cood = {
  {400, 400, 120, 240}, //{400, 400, 120, 120}
  {640, 400, 120, 240},
  {780, 400, 120, 120},
  {780, 540, 120, 120},
  {780, 680, 120, 120}
};

String [] messages_received = {
  " ", " ", " ", " ", " ",
  " ", " ", " ", " ", " " };
int message_index = 0;

int time_ant1;
boolean blink1 = true;
int time_ant2;
boolean blink2 = true;

void setup()
{
  fullScreen();
  //size(300, 300);

  mqtt = new MQTTClient(this);
  mqtt.connect(MQTT_BROKER, MQTT_CLIENT_ID);
  time_ant1 = millis();
  time_ant2 = millis();
}

void draw ()
{
  background(AZUL_MARINO); // Sin esto no funcionan bien las animaciones

  circuit_breaker_displays(displayWidth * 9 / 11, displayHeight * 2 / 11, "VOLTAGE", "CURRENT", rx_PIOV_state, rx_PIOC_state, rx_XCBR_state);

  fill(255);
  textSize(50);
  textAlign(CENTER, CENTER);
  text("XCBR", 460, 350);
  text("TV/TCTR", 770, 350);
  interruptor(interac_cood[0][0], interac_cood[0][1], interac_cood[0][2], interac_cood[0][3], 0, "OPEN", "CLOSE");
  interruptor(interac_cood[1][0], interac_cood[1][1], interac_cood[1][2], interac_cood[1][3], 1, "START", "STOP");
  button(interac_cood[2][0], interac_cood[2][1], interac_cood[2][2], interac_cood[2][3], YELLOW, 2, "VOLTAGE/\nCURRENT");
  button(interac_cood[3][0], interac_cood[3][1], interac_cood[4][2], interac_cood[3][3], YELLOW, 3, "VOLTAGE");
  button(interac_cood[4][0], interac_cood[4][1], interac_cood[4][2], interac_cood[4][3], YELLOW, 4, "CURRENT");
  message_displayer();


  /*rect();
   rectMode();*/
}

void mouseClicked()
{
  for (int i = 0; i < interac_cood.length; i++)
  {
    if (mouseX >= interac_cood[i][0] && mouseX <= interac_cood[i][0] + interac_cood[i][2] && mouseY >= interac_cood[i][1] && mouseY <= interac_cood[i][1] + interac_cood[i][3])
    {
      switch(i)
      {
      case 0:
        last_state_XCBR = current_state_XCBR;
        if (mouseY <= interac_cood[i][1] + interac_cood[i][3]  / 2 && !current_state_XCBR)
        {
          current_state_XCBR = true;
          interac[i][1] = true;
          mqtt.publish(INST_XCBR, "open");
        } else if (mouseY > interac_cood[i][1] + interac_cood[i][3]  / 2 && current_state_XCBR)
        {
          current_state_XCBR = false;
          interac[i][1] = false;
          mqtt.publish(INST_XCBR, "close");
        }
        break;
      case 1:
        last_state_TC_TVTR = current_state_TC_TVTR;

        if (mouseY <= interac_cood[i][1] + interac_cood[i][3]  / 2 && current_state_TC_TVTR)
        {
          current_state_TC_TVTR = false;
          interac[i][1] = true;
          mqtt.publish(INST_TC_TVTR_st, "start");
        } else if (mouseY > interac_cood[i][1] + interac_cood[i][3]  / 2 && !current_state_TC_TVTR)
        {
          current_state_TC_TVTR = true;
          interac[i][1] = false;
          mqtt.publish(INST_TC_TVTR_st, "stop");
        }
        break;
      case 2:
        if (!interac[i][1])
        {
          interac[i][1] = true;
          interac[3][1] = false;
          interac[4][1] = false;
          mqtt.publish(INST_TC_TVTR_ty, "all");
        }
        break;
      case 3:
        if (!interac[i][1])
        {
          interac[i][1] = true;
          interac[2][1] = false;
          interac[4][1] = false;
          mqtt.publish(INST_TC_TVTR_ty, "voltage");
        }

        break;
      case 4:
        if (!interac[i][1])
        {
          interac[i][1] = true;
          interac[2][1] = false;
          interac[3][1] = false;
          mqtt.publish(INST_TC_TVTR_ty, "current");
        }
        break;
      }
    }
  }
}

void mouseMoved()
{
  for (int i = 0; i < interac_cood.length; i++)
  {
    if (mouseX >= interac_cood[i][0] && mouseX <= interac_cood[i][0] + interac_cood[i][2] && mouseY >= interac_cood[i][1] && mouseY <= interac_cood[i][1] + interac_cood[i][3])
    {
      interac[i][0] = true;
    } else
    {
      interac[i][0] = false;
    }
  }
}

void mouseReleased()
{
  for (int i = 0; i < interac_cood.length; i++)
  {
    if (interac[i][1] == true && mouseX < interac_cood[i][0] && mouseX > interac_cood[i][0] + interac_cood[i][2]
      && mouseY < interac_cood[i][1] && mouseY > interac_cood[i][1] + interac_cood[i][3])
    {
      interac[i][1] = false;
    }
  }
}

void clientConnected()
{
  println("MQTT client connected.");
  mqtt.subscribe(TOPIC_SUBS);
}

void messageReceived(String topic, byte[] payload)
{
  int aux = millis();
  String temp =  new String();

  if (topic.equals(ORD_XCBR))
  {
    rx_XCBR_state = boolean(new String(payload));
    last_state_XCBR = current_state_XCBR;
    current_state_XCBR = rx_XCBR_state;
  } else if (topic.equals(ORD_PIOC))
  {
    rx_PIOC_state = boolean(new String(payload));
  } else if (topic.equals(ORD_PIOV))
  {
    rx_PIOV_state = boolean(new String(payload));
  } else if (topic.equals(ORD_TC_TVTR))
  {
    rx_TC_TVTR_values = bytesToFloatArray(payload);
  }

  for (int i = messages_received.length - 1; i > 0; i--)
  {
    messages_received[i] = messages_received[i - 1];
  }
  if (topic.equals(ORD_TC_TVTR))
  {
    temp = String.format("%d:%02d:%02d,%03d   %s", aux / 3600000, (aux / 60000) % 60,
      (aux / 1000) % 60, aux % 1000,
      topic);
  } else
  {
    temp = String.format("%d:%02d:%02d,%03d   %s --> %s", aux / 3600000, (aux / 60000) % 60,
      (aux / 1000) % 60, aux % 1000,
      topic, new String(payload));
  }

  messages_received[0] = temp;
}

void connectionLost() {
  println("MQTT connection lost.");
}

void interactive_square(int x, int y, int s_width, int s_height)
{
  noFill();
  cuadrado_relieve(x, y, s_width, s_height, -1);
}

void interruptor(int x, int y, int s_width, int s_height, int i, String text1, String text2)
{
  // Cuadrado
  rectMode(CORNER);
  if (interac[i][0]) {
    interactive_square(x - 7, y - 7, s_width + 14, s_height + 14);
  }
  // Colores del interruptor
  fill(RED);
  rect(x, y, s_width, s_height / 2);
  fill(GREEN);
  rect(x, y + s_height / 2, s_width, s_height / 2);
  // Cuadrado
  cuadrado_relieve(x, y, s_width, s_height, -1);

  noStroke();
  if (interac[i][1])
  {
    // Sombreado al presionar
    if (i == 0) {
      sombreado2(x + 4, y + 4, s_width - 7, (s_height - 7), !current_state_XCBR);
    } else if (i == 1) {
      sombreado2(x + 4, y + 4, s_width - 7, (s_height - 7), current_state_TC_TVTR);
    }
    // Texto del botón
    textAlign(CENTER, CENTER);
    textSize(20);
    fill(255, 124);
    text(text1, x + (s_width / 2) + 2, y + (s_height / 4) + 2);
    fill(124);
    text(text1, x + s_width / 2, y + s_height / 4);
    text(text2, x + (s_width / 2) + 2, y + (s_height * 3 / 4) + 2);
    fill(0);
    text(text2, x + s_width / 2, y + s_height * 3 / 4);
  } else
  {
    // Sombreado cuando no se presiona
    if (i == 0) {
      boolean b = !current_state_XCBR;
      sombreado2(x + 4, y + 4, s_width - 7, (s_height - 7), b);
    } else if (i == 1) {
      sombreado2(x + 4, y + 4, s_width - 7, (s_height - 7), current_state_TC_TVTR);
    }
    textAlign(CENTER, CENTER);
    textSize(20);
    fill(124);
    text(text1, x + (s_width / 2) + 2, y + (s_height / 4) + 2);
    fill(0);
    text(text1, x + s_width / 2, y + s_height / 4);
    fill(255, 124);
    text(text2, x + (s_width / 2) + 2, y + (s_height * 3 / 4) + 2);
    fill(124);
    text(text2, x + s_width / 2, y + s_height * 3 / 4);
  }
}

void button(int x, int y, int s_width, int s_height, int colour, int i, String text)
{
  // Cuadrado
  rectMode(CORNER);
  if (interac[i][0]) {
    interactive_square(x - 7, y - 7, s_width + 14, s_height + 14);
  }
  // Color del botón
  fill(colour);
  // Cuadrado
  cuadrado_relieve(x, y, s_width, s_height, colour);

  noStroke();
  if (interac[i][1])
  {
    // Sombreado al presionar
    sombreado(x + 4, y + 4, s_width - 7, s_height - 7, interac[i][1]);
    // Texto del botón
    textAlign(CENTER, CENTER);
    textSize(20);
    fill(255, 124);
    text(text, x + (s_width / 2) + 2, y + (s_height / 2) + 2);
    fill(124);
    text(text, x + s_width / 2, y + s_height / 2);
  } else
  {
    // Sombreado cuando no se presiona
    sombreado(x + 4, y + 4, s_width - 7, s_height - 7, interac[i][1]);
    // Texto del botón
    textAlign(CENTER, CENTER);
    textSize(20);
    fill(124);
    text(text, x + (s_width / 2) + 2, y + (s_height / 2) + 2);
    fill(0);
    text(text, x + s_width / 2, y + s_height / 2);
  }
}

void circuit_breaker_displays(int x, int y, String text1, String text2, boolean display_state1, boolean display_state2, boolean b_state)
{
  circuit_breaker(x, y, b_state);
  trafo(x, y + 404);

  sensor_graphic(x, y + 249);
  sensor_tags_display(x - 520, y + 90, 180, 130, text1, 0);
  sensor_tags_display(x - 520, y + 483, 180, 130, text2, 1);
  sensor_alarm_display(x - 370, y + 48, 30, 30, display_state1, 0);
  sensor_alarm_display(x - 370, y + 441, 30, 30, display_state2, 1);
}

void trafo (int x, int y)
{
  noFill();
  stroke(255);
  strokeWeight(3);
  // Círculos
  circle(x, y, 200);
  circle(x, y + 148, 200);
  // Líneas extremos
  line(x, y - 200, x, y - 100);
  line(x, y + 248, x, y + 348);
}

void circuit_breaker (int x, int y, boolean state)
{
  // Línea de los extremos y puntos
  stroke(255);
  fill(255);
  strokeWeight(3);
  circle(x, y, 10);
  circle(x, y + 200, 10);
  // Línea del interruptor
  if (!state)
  { // Cerrado
    line(x, y - 50, x, y + 200);
  } else
  { // Abierto
    line(x, y - 50, x, y + 50);
    line(x - 57, y + 68, x, y + 150);
    line(x, y + 150, x, y + 200);
  }
}

void sensor_graphic (int x, int y)
{
  // Cuadrado
  rectMode(CENTER);
  stroke(255);
  noFill();
  strokeWeight(3);
  rect(x, y, 40, 40);
  // Líneas
  line(x - 20, y, x - 190, y);
  line(x - 190, y, x - 190, y - 94);
  line(x - 190, y - 94, x - 340, y - 94);
  line(x - 190, y, x - 190, y + 299);
  line(x - 190, y + 299, x - 340, y + 299);
}

void sensor_tags_display (int x, int y, int s_width, int s_height, String text, int type)
{
  // Título
  textAlign(LEFT, BOTTOM);
  textSize(30);
  fill(255);
  text(text, x, y - 10);
  // Cuadro
  rectMode(CORNER);
  fill(250);
  cuadrado_relieve(x, y, s_width, s_height, 250);
  //Texto de celdas
  textAlign(LEFT, CENTER);
  textSize(20);
  fill(0);
  //Celdas y contenido del texto
  for (int i = 1; i < 8; i++)
  {
    if (i % 2 == 0) {
      stroke(0);
      line(x + 3, y + s_height * i / 8 - 2, x + s_width - 3, y + s_height * i / 8 - 2);
      stroke(255);
      line(x + 3, y + s_height * i / 8 - 1, x + s_width - 3, y + s_height * i / 8 - 1);
      stroke(126);
      line(x + 3, y + s_height * i / 8, x + s_width - 3, y + s_height * i / 8);
      stroke(0);
      line(x + 3, y + s_height * i / 8 + 1, x + s_width - 3, y +s_height * i / 8 + 1);
    } else {
      print_tags_display(x, y, s_height, i, type);
    }
  }
}

void print_tags_display(int x, int y, int s_height, int i, int type)
{
  String eng_units = "";
  String variable = "";

  if (type == 0)
  {
    switch(i)
    {
    case 1:
      variable = "VA:  ";
      break;
    case 3:
      variable = "VB:  ";
      break;
    case 5:
      variable = "VC:  ";
      break;
    case 7:
      variable = "VN:  ";
      break;
    }

    eng_units = " V";

    text(variable + nf(rx_TC_TVTR_values[i / 2], 1, 3) + eng_units, x + 10, y + s_height * i / 8);
  } else if (type == 1)
  {
    switch(i)
    {
    case 1:
      variable = "IA:  ";
      break;
    case 3:
      variable = "IB:  ";
      break;
    case 5:
      variable = "IC:  ";
      break;
    case 7:
      variable = "IN:  ";
      break;
    }

    eng_units = " A";

    text(variable + nf(rx_TC_TVTR_values[(i / 2) + 4], 1, 3) + eng_units, x + 10, y + s_height * i / 8);
  }
}

void sensor_alarm_display (int x, int y, int s_width, int s_height, boolean b_state, int j)
{
  // Colores de relleno
  int state = b_state ? 1 : 0;
  noStroke();
  switch (state) {
  case 0:
    fill(GREEN);
    break;
  case 1:
    if (j == 0) {
      if (timer(time_ant1, 500))
      {
        blink1 = !blink1;
        time_ant1 = millis();
      }
    } else if (j == 1) {
      if (timer(time_ant2, 500))
      {
        blink2 = !blink2;
        time_ant2 = millis();
      }
    }

    if ((blink1 && j == 0) || (blink2 && j == 1))
    {
      fill(RED);
    } else
    {
      fill(50);
    }
    break;
  default:
    fill(50);
    break;
  }
  rect(x, y, s_width, s_height);
  sombreado(x + 4, y + 4, s_width - 7, s_height - 7, false);
  // Cuadrado
  cuadrado_relieve(x, y, s_width, s_height, -1);
}

void message_displayer()
{
  fill(255);
  noStroke();
  rect(0, displayHeight - 300, 550, 300);
  fill(100);
  rect(0, displayHeight - 300, 550, 30);
  fill(240);
  textAlign(LEFT, BOTTOM);
  textSize(20);
  text("MQTT Mesages Received", 10, displayHeight - 274);
  fill(0);

  for (int i = 0; i < messages_received.length; i++)
  {
    text(messages_received[i], 10, displayHeight - 247 + 27 * i);
  }

  cuadrado_relieve(0, displayHeight - 300, 550, 325, -1);
}

void sombreado(int x, int y, int s_width, int s_height, boolean mode)
{
  if (mode)
  {
    fill(240, 100);
    quad(x + s_width * 0.15, y + s_height * 0.85,
      x + s_width * 0.85, y + s_height * 0.85,
      x + s_width, y + s_height,
      x, y + s_height);
    fill(200, 100);
    quad(x + s_width * 0.85, y + s_height * 0.85,
      x + s_width * 0.85, y + s_height * 0.15,
      x + s_width, y,
      x + s_width, y + s_height);
    fill(140, 100);
    quad(x, y + s_height,
      x, y,
      x + s_width * 0.15, y + s_height * 0.15,
      x + s_width * 0.15, y + s_height * 0.85);
    fill(100, 100);
    quad(x, y,
      x + s_width, y,
      x + s_width * 0.85, y  + s_height * 0.15,
      x + s_width * 0.15, y  + s_height * 0.15);
    fill(40, 100);
    rect(x + s_width * 0.15, y  + s_height * 0.15, s_width * 0.7, s_height * 0.7);
  } else
  {
    fill(100, 100);
    quad(x + s_width * 0.15, y + s_height * 0.85,
      x + s_width * 0.85, y + s_height * 0.85,
      x + s_width, y + s_height,
      x, y + s_height);
    fill(140, 100);
    quad(x + s_width * 0.85, y + s_height * 0.85,
      x + s_width * 0.85, y + s_height * 0.15,
      x + s_width, y,
      x + s_width, y + s_height);
    fill(200, 100);
    quad(x, y + s_height,
      x, y,
      x + s_width * 0.15, y + s_height * 0.15,
      x + s_width * 0.15, y + s_height * 0.85);
    fill(240, 100);
    quad(x, y,
      x + s_width, y,
      x + s_width * 0.85, y  + s_height * 0.15,
      x + s_width * 0.15, y  + s_height * 0.15);
  }
}

void sombreado2(int x, int y, int s_width, int s_height, boolean mode)
{
  if (mode)
  {
    // Sombreado pulsado superior
    fill(200, 100);
    quad(x + s_width * 0.85, y + s_height / 2,
      x + s_width * 0.85, y + s_height * 0.15 / 2,
      x + s_width, y,
      x + s_width, y + s_height / 2);
    fill(140, 100);
    quad(x, y + s_height / 2,
      x, y,
      x + s_width * 0.15, y + s_height * 0.15 / 2,
      x + s_width * 0.15, y + s_height / 2);
    fill(100, 100);
    quad(x, y,
      x + s_width, y,
      x + s_width * 0.85, y + s_height * 0.15 / 2,
      x + s_width * 0.15, y + s_height * 0.15 / 2);
    fill(40, 100);
    rect(x + s_width * 0.15, y + s_height * 0.15 / 2, s_width * 0.69 + 0.5, s_height * 0.85 / 2 - 0.5);
    // Sombreado sin pulsar inferior
    fill(100, 100);
    quad(x + s_width * 0.15, y + s_height * 0.925,
      x + s_width * 0.85, y + s_height * 0.925,
      x + s_width, y + s_height,
      x, y + s_height);
    fill(140, 100);
    quad(x + s_width * 0.85, y + s_height * 0.925,
      x + s_width * 0.85, y + s_height / 2,
      x + s_width, y + s_height / 2,
      x + s_width, y + s_height);
    fill(200, 100);
    quad(x, y + s_height,
      x, y + s_height / 2,
      x + s_width * 0.15, y + s_height / 2,
      x + s_width * 0.15, y + s_height * 0.925);
  } else
  {
    // Sombreado sin pulsar superior
    fill(100, 100);
    quad(x + s_width * 0.85, y + s_height / 2,
      x + s_width * 0.85, y + s_height * 0.15 / 2,
      x + s_width, y,
      x + s_width, y + s_height / 2);
    fill(140, 100);
    quad(x, y + s_height / 2,
      x, y,
      x + s_width * 0.15, y + s_height * 0.15 / 2,
      x + s_width * 0.15, y + s_height / 2);
    fill(200, 100);
    quad(x, y,
      x + s_width, y,
      x + s_width * 0.85, y + s_height * 0.15 / 2,
      x + s_width * 0.15, y + s_height * 0.15 / 2);
    // Sombreado pulsado inferior
    fill(200, 100);
    quad(x + s_width * 0.15, y + s_height * 0.925,
      x + s_width * 0.85, y + s_height * 0.925,
      x + s_width, y + s_height,
      x, y + s_height);
    fill(140, 100);
    quad(x + s_width * 0.85, y + s_height * 0.925,
      x + s_width * 0.85, y + s_height / 2,
      x + s_width, y + s_height / 2,
      x + s_width, y + s_height);
    fill(100, 100);
    quad(x, y + s_height,
      x, y + s_height / 2,
      x + s_width * 0.15, y + s_height / 2,
      x + s_width * 0.15, y + s_height * 0.925);
    fill(40, 100);
    rect(x + s_width * 0.15, y + s_height / 2, s_width * 0.69 + 0.5, s_height * 0.85 / 2 - 0.5);
  }
}

void cuadrado_relieve(int x, int y, int s_width, int s_height, int colour)
{
  if (colour == -1)
  {
    noFill();
  } else
  {
    fill(colour);
  }

  strokeWeight(1);

  stroke(0);
  rect(x, y, s_width, s_height);

  stroke(255);
  strokeWeight(2);
  rect(x + 2, y + 2, s_width - 3, s_height - 3);

  stroke(0);
  strokeWeight(1);
  rect(x + 3, y + 3, s_width - 6, s_height - 6);

  stroke(126);
  line(x + 2, y + 2, x + 2, y + s_height - 3);
  line(x + 2, y + 2, x + s_width - 3, y + 2);
  line(x + 2, y + 2, x + 2, y + s_height - 3);
  line(x + 1, y + s_height - 1, x + s_width - 2, y + s_height - 1);
  line(x + s_width - 1, y + 1, x + s_width - 1, y + s_height - 1);
}

boolean timer(int time_ant, int t_ms) {
  if (millis() - time_ant >= t_ms)
  {
    return true;
  }
  return false;
}



public float[] bytesToFloatArray(byte[] payload) {
  // Verifica que el tamaño sea múltiplo de 4
  if (payload.length % 4 != 0) {
    throw new IllegalArgumentException("El tamaño del payload no es válido para float[]");
  }

  int numFloats = payload.length / 4;
  float[] floats = new float[numFloats];

  // Crear ByteBuffer con el array de bytes
  ByteBuffer buffer = ByteBuffer.wrap(payload);

  // Asegurarse del orden de bytes (endianness)
  buffer.order(ByteOrder.LITTLE_ENDIAN); // o BIG_ENDIAN, depende de cómo se envió

  // Leer floats del buffer
  for (int i = 0; i < numFloats; i++) {
    floats[i] = buffer.getFloat();
  }

  return floats;
}
