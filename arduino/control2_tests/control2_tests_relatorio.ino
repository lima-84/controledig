/*

   Código utilizado para amostragem de um sistema.

   Gera uma sequência de números aleatórios segundo um algoritmo de xorshift.
   Aplica essa sequência ao sistema com um período especificado.
   Popula vetores com as amostras medidas e manda por serial quando finalizado.
   É enviado por serial um valor de 0-255 para os inputs aplicados e um valor de
   0-1023 para as saídas medidas (para poupar memória).
   Inputs repetidos são omitidos.

Autor: Pedro Sidra de Freitas
*/

// Definições do usuário pré-compilação:
const int BATCH_SIZE = 200;
const int NUM_INPUTS = (BATCH_SIZE / 20 ); // alterar o divisor para alterar frequência
const int INPUT_CHANGE_PERIOD = BATCH_SIZE/NUM_INPUTS;
// Usar PRBS ou não
const bool PRBS = false;

// vetores amostrados:
int sample_buffers[BATCH_SIZE] = {};
uint16_t input_list[NUM_INPUTS] = {};

// Variáveis de programa
char timer_count = 0;
bool sample = false;
bool send_serial = false;
uint16_t sample_count = 0;
uint16_t input_count = 0;
char received_from_serial = '0';

// Gerador de números pseudo-aleatórios
unsigned int s[2] = {1231, 999999};
unsigned int rng()
{
    unsigned int x = s[0];
    unsigned int y = s[1];
    s[0] = y;
    x ^= x << 7; // a
    s[1] = x ^ y ^ (x >> 3) ^ (y >> 5); // b, c

    // se PRBS, retorna (0,1)*255
    if (PRBS)
        return ((s[1] + y)%2)*255;
    else
        return (s[1] + y)%255;
}

// Envia as amostras por serial
void send_samples()
{
    int i=0;
    for (i=0; i < BATCH_SIZE; i++){
        Serial.println(sample_buffers[i]);
    }
    Serial.println("inputs");
    for (int i=0; i < NUM_INPUTS; i++)
        Serial.println(input_list[i]);
}

// Função de interrupção
ISR(TIMER2_COMPA_vect)
{
    // Corta a frequência por um fator de 10
    timer_count++;
    if (timer_count>9)
    {
        timer_count=0;
        if(sample)
        {
            // Grava y
            sample_buffers[sample_count] = analogRead(A0);
            // altera o input quando necessário
            if (sample_count % INPUT_CHANGE_PERIOD == 0)
            {
                analogWrite(9, input_list[input_count]);
                input_count = (input_count + 1)%NUM_INPUTS;
            }
            sample_count++;
            // sinaliza término da amostragem
            if (sample_count==BATCH_SIZE)
            {
                sample_count =0;
                send_serial = true;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Altera a frequência do PWM na porta 9
    // para a máxima possível
    TCCR1B = (TCCR1B & 0b11111000) | 0x01;

    // Seta o timer 2 para interrupts a 10khz
    TCCR2A = 0; TCCR2B = 0; TCNT2  = 0;
    OCR2A = 199;// = (16*10^6) / (10e3*8) - 1 
    TCCR2A |= (1 << WGM21); TCCR2B |= (1 << CS21);
    TIMSK2 |= (1 << OCIE2A);

    for(int i=0; i < NUM_INPUTS; i++)
    {
        input_list[i] = rng();
    }

    analogWrite(9, 0);
}
void loop() {

    if (send_serial == true)
    {
        sample=false;
        send_serial = false;
        send_samples();
    }

    if (Serial.available()){
        received_from_serial = Serial.read();
        switch (received_from_serial){
            case 'b':
                sample=true;
                break;
            case 'q':
                sample=false;
                analogWrite(9, 0);
                break;
            case 'r':
                Serial.println(rng());
                break;
        }
        received_from_serial = '0';
    }
}
