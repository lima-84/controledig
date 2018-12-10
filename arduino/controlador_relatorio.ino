/*
    Implementa o controle PID a partir de ganhos calculados 
    por VDFT. 

    PID é da forma kp*e + ki*i + kd * d,
    com:
    i(t) = e(t) + i(t-1)
    d(t) = d(t) - d(t-1)

Autor: Pedro Sidra de Freitas

*/

// Definição dos ganhos:
#define KP 1.143677268970767
#define KI 0.4913055708142872 
#define KD 0.0964895917972739

const int BATCH_SIZE = 200;
int y[BATCH_SIZE] = {};
float u[BATCH_SIZE] = {};

// Variáveis de programa
char timer_count = 0;
bool send_serial = false;
bool on = false;
uint16_t sample_count = 0;
float utemp =0;
char received_from_serial = '0';

// Definições do controle
float ref =2.5, e=0, eant=0, i=0, d=0;
// perturbação aplicada
float perturb = 0;


// Envia amostras coletadas para validação do controlador
void send_samples()
{
    int i=0;
    for (i=0; i < BATCH_SIZE; i++){
        Serial.println(y[i]);
    }
    Serial.println("inputs");
    for (int i=0; i < BATCH_SIZE; i++)
        Serial.println(u[i]);
}

// Função de interrupt
ISR(TIMER2_COMPA_vect)
{
    // corta a frequência de amostragem
    timer_count++;
    if (timer_count>9)
    {
        timer_count=0;
        if(on)
        {
            // Calcula phi(t)
            eant = e;
            y[sample_count] = analogRead(A0);
            e = ref - (float)y[sample_count]*5.0/1023.0;
            i = e + i;
            d = e - eant;
            // Calcula u = theta*phi(t)
            // 51 = 255/5, fator de escala para aplicação na função
            // analogWrite
            u[sample_count] = 51*((KP*e + KI*i + KD*d));

            // Ao aplicar a saída do controlador, gerar uma perturbação
            utemp = (51*perturb + u[sample_count]) ;

            if (utemp>255)
                utemp = 255;
            if (utemp<0)
                utemp = 0;

            analogWrite(9, utemp);

            sample_count ++;
            // Altera a perturbação a cada 25 amostras a partir 
            // das primeiras 25
            if ((sample_count % 25) == 0)
            {
                if (perturb == 0) perturb=-0.5;
                perturb *= -1;
            }
            if (sample_count == BATCH_SIZE)
            {
                sample_count=0;
                send_serial = true;
                on = false;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    // Altera a frequência do pwm na porta 9 para a máxima possível
    TCCR1B = (TCCR1B & 0b11111000) | 0x01;

    // gera interrupções no timer2 a 10khz
    TCCR2A = 0; TCCR2B = 0; TCNT2  = 0;
    OCR2A = 199;// = (16*10^6) / (10e3*8) - 1 
    TCCR2A |= (1 << WGM21); TCCR2B |= (1 << CS21);
    TIMSK2 |= (1 << OCIE2A);

    analogWrite(9, 0);
}
void loop() {

    if (send_serial == true)
    {
        on = false;
        send_serial = false;
        send_samples();
    }

    if (Serial.available()){
        received_from_serial = Serial.read();
        switch (received_from_serial){
            case 'b':
                on = true;
                break;
            case 'q':
                on = false;
                analogWrite(9, 0);
                break;
        }
        received_from_serial = '0';
    }
}
