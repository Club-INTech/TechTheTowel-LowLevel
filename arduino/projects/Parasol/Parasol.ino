// Ouverture du parasol (robot secondaire)
// Transistor Moteur PIN 7 ; jumper relié au PIN 8 ; led de "match en cours" pin 4 ; capteur de contact pin 6
// Moteur alimenté avec pile 9V et transistor polarisé amplificateur

bool done = false;
long int t_depart = 0;


void setup() {
  pinMode(7, OUTPUT);
  pinMode(8, INPUT);
  pinMode(4, OUTPUT);
  pinMode(3, INPUT);
  digitalWrite(7, LOW);
  
 
}

void loop() {
  bool blink = false;
  
   if(digitalRead(8) && !done) { // On attend que le jumper soit mis en place (utile pour déterminer un front descendant, duh...)
      digitalWrite(4, HIGH); // On indique qu'il a compris que le match commence
     while(digitalRead(8)) {
       // break;   <--- Utile pour tests sans jumper
     } // On attends le front descendant (enlevage du jumper)

     
     t_depart = millis();
     
     for(int i=0; i<=84; i++){ // 85 secondes
     
      delay(1000);  // Oui, c'est dégeulasse.
      blink = !blink;
      digitalWrite(4, blink);
     }

     for(int i=0; i<=19; i++){ // 5 dernières secondes
      delay(250);
      blink = !blink;
      digitalWrite(4, blink);
     }
     
     
     if((millis() - t_depart) <= 95000 && (millis() - t_depart) >= 90000) { // Empêche le lancement du moteur si le temps est écoulé ou s'il est trop tôt (overkill mais on ne l'est jamais trop quand il s'agit de ne pas se prendre une pénalité de 20 points)
      digitalWrite(7, HIGH);
      while(42)
      {
        if(digitalRead(3)) 
        {
          digitalWrite(7, LOW);
          break;
        }
      }
     }

     digitalWrite(4, LOW);
     done=true; // Empeche le système de se relancer
   }
  
}
