// Timer frequency = 10 kHz
// Timer period = 1.0e-4 s = 100 us

// dominant pole: s = -355.196886
// ts = 0.01126136

// pwm frequency = 31372.55 Hz 
// pwm period = 31.875 us

const int BATCH_SIZE = 200;
const int NUM_INPUTS = (BATCH_SIZE / 5 );
const int INPUT_CHANGE_PERIOD = BATCH_SIZE/NUM_INPUTS; // in samples

int sample_buffers[BATCH_SIZE] = {};
uint16_t input_list[NUM_INPUTS] = {};

const bool DEBUG = false;
const bool PRBS = true;

char timer_count = 0;
bool sample = false;
bool send_serial = false;
uint16_t sample_count = 0;
uint16_t input_count = 0;

char received_from_serial = '0';

// Random number generator based on xorshift 
// (adapted for fewer number of bits)
unsigned int s[2] = {1231, 999999};
unsigned int rng()
{
    unsigned int x = s[0];
    unsigned int y = s[1];
    s[0] = y;
    x ^= x << 7; // a
    s[1] = x ^ y ^ (x >> 3) ^ (y >> 5); // b, c

    if (PRBS)
        return ((s[1] + y)%2)*255;
    else
        return (s[1] + y)%255;
}

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

ISR(TIMER2_COMPA_vect)
{
    timer_count++;
    if (timer_count>10)
    {
        timer_count=0;
        if(sample)
        {
            sample_buffers[sample_count] = analogRead(A0);
            if (sample_count % INPUT_CHANGE_PERIOD == 0)
            {
                analogWrite(9, input_list[input_count]);
                input_count = (input_count + 1)%NUM_INPUTS;
            }
            sample_count++;
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
    // Timer secrets: https://playground.arduino.cc/Main/TimerPWMCheatsheet
    // https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
    TCCR1B = (TCCR1B & 0b11111000) | 0x01;

    //set timer2 interrupt at 8kHz
    TCCR2A = 0; TCCR2B = 0; TCNT2  = 0;
    // set compare match register for 10khz increments
    OCR2A = 199;// = (16*10^6) / (10e3*8) - 1 (must <256)
    // turn on CTC mode, Set CS21 bit for 8 prescaler
    TCCR2A |= (1 << WGM21); TCCR2B |= (1 << CS21);
    // enable timer compare interrupt
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
