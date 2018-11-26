#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" //Librairie plus complète que MPU6050.h pour quaternion
#include "NewPing.h" //Librairie pour l'utilisation du senseur ultrasonique HC-SR04

//Constantes et variables pour la lecture des données du MPU-6050 et corrections
const short nbLecturesCompteurs = 20; //Nombre de lecture que l'Arduino doit faire avant de pouvoir dire que
                                      //la valeur est stable et que le MPU ne bouge pas
const short delai = 1;  //Constante ayant le délai souhaité dans la boucle.
const float sensibiliteAccel = 16384.0; //Selon la documentation du senseur, pour une valeur maximum de 2g
const float sensibiliteVitesse = 131.0; //Selon la documentation du senseur, pour une valeur maximum de 250 degrés/seconde
const float erreurAccelXY = 0.1f, erreurAccelXYmax = 0.1f;  //Erreur d'arrondissement pour les accélérations en x et y
const float erreurAccelZ = 0.05f, erreurAccelZmax = 0.05f;  //Erreur d'arrondissement pour l'accélération en z
const float erreurVitesse = 0.5f;  //Erreur d'arrondissement pour les vitesses angulaires
const float erreurHauteur = 1.0f;  //Erreur d'arrondissement pour la hauteur
short tabCompteurs[7];//tabCompteurs[0]: compteurAX, tabCompteurs[1]: compteurAY, tabCompteurs[2]: compteurAZ, 
                      //tabCompteurs[3]: compteurVX, tabCompteurs[4]: compteurVY, tabCompteurs[5]: compteurVZ
                      //tabCompteurs[6]: compteurHauteur
//--------------------------------------------------------------------------------------------------------------------------
//Variables pour l'envoi de données vers Processing 
//Code en partie basée sur celui du MPU Teapot
bool dmpReady = false;  // True si l'initialisation du DMP se fait correctement, pour ne pas avoir de bugs
volatile bool mpuInterrupt = false;  // Fonction : Si le DMP est prêt, retourne vrai. Sinon, faux par défaut
uint8_t iStatutMPU;   // Contient le statut d'interruption du MPU en Byte
uint8_t iStatutDMP;      // Contient le statut du DMP après chaque opératon (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // tampon FIFO. Recueille les informations du FIFO
uint16_t taillePacket;    // Contient la grosseur expectée du packet DMP (42 bytes par défaut)
uint16_t nbByteFIFO;     // Contient le compte de tous les bytes dans le FIFO
int16_t axT, ayT, azT, vxT, vyT, vzT;   //Valeurs temporaires des 6 données provenant du senseur
float ax,ay,az;   //Accélérations mesurées en x, y et z
float vx,vy,vz;   //Vitesses angulaires mesurées en x, y et z
float euler[3];  // [psi, theta, phi]    tableau contenant les 3 angles d'Euler
Quaternion quat1; // [w, x, y, z]         conteneur de quaternion
//--------------------------------------------------------------------------------------------------------------------------
//Variables pour la lecture du senseur ultrasonique
#define ENVOI_PIN 8 //Pin utilisée pour l'envoi
#define RETOUR_PIN 7
#define MAX_DISTANCE 200
const float VITESSE_SON = 340.0 / 10000;//Vitesse du son dans l'air en cm/us
float duree = 0.0f; //Mesure de la hauteur à partir du senseur ultrasonique
float distanceSol = 0.0f; //Distance du gant par rapport au sol
float distanceSolT = 0.0f; ////Distance temporaire du gant par rapport au sol
float fIntervalleSonar = -50; //Intervalle de temps depuis la dernière prise de donnée du sonar
MPU6050 senseurMPU; //0x68 : high / Fait appel au MPU6050, ses fonctions nous donnent des données
NewPing sonar(ENVOI_PIN, RETOUR_PIN, MAX_DISTANCE);

//Initialisation du MPU, des librairies et des offsets sur le MPU.
void setup() 
{
  Wire.begin(); //Initialise la librairie Wire et joint I2C avec elle. 

  Serial.begin(115200); // liaison série avec une fréquence de 155'200 Baud.
  
  //Initialisation des pins du senseur ultrasonique
  pinMode(ENVOI_PIN, OUTPUT);
  digitalWrite(RETOUR_PIN, LOW); // La pin ENVOI doit être à LOW au repos
  pinMode(RETOUR_PIN, INPUT);
  
  while (!Serial) {}  //Si le port série n'est pas connecté/initialisé.
  senseurMPU.initialize();  // initialisation du MPU-6050.
  iStatutDMP = senseurMPU.dmpInitialize(); // Va chercher le statut du DMP (0 = success, !0 = error)
  statutDMP();
  
  // Calibration du senseur MPU en éliminant les décalages/offsets:
  // ces offsets proviennent des résultats d'un différent sketch Arduino (voir documentation, Glossaire)
  senseurMPU.setXAccelOffset(309);   
  senseurMPU.setYAccelOffset(2773);  
  senseurMPU.setZAccelOffset(1421);
  senseurMPU.setXGyroOffset(-218);
  senseurMPU.setYGyroOffset(-32);
  senseurMPU.setZGyroOffset(15);
  
}

//Appelle les fonctions principales à chaque boucles
void loop() 
{  
  if (!dmpReady) return;  // Si la configuration n'a pas fonctionné, la boucle ne sera pas utilisé
  {    
    statutMPU();
    
    senseurMPU.dmpGetQuaternion(&quat1, fifoBuffer); //Crée un quaternion des données du fifoBuffer
    senseurMPU.dmpGetEuler(euler, &quat1); //Crée les angles Euler à partir du quaternion
    
    senseurMPU.getAcceleration(&axT, &ayT, &azT);  //Prend les accélérations de rotation du MPU6050
    senseurMPU.getRotation(&vxT, &vyT, &vzT);  //Prend les vitesses de rotation du MPU6050
   
    priseDonneeHauteur();   
    restrictionMargesErreur();
    ecritureDonnee();    
  }
}

void dmpDataReady() 
{
    mpuInterrupt = true;
}

//Prise de donnée du senseur ultrasonique: assignation de la hauteur du gant relative au sol
void priseDonneeHauteur()
{
  if(millis() - fIntervalleSonar >= 50)
  {
    float testDistanceSol = sonar.ping_cm();  //On prend la mesure de la distance
    if(testDistanceSol != 0)  //Si elle n'égale pas 0, on la prend en considération
    { //On la stocke si la valeur est bonne (si c'est 0, c'est, la plupart du temps, 
      // à cause que l'onde n'est pas revenu
      distanceSolT = testDistanceSol;
      erreurMesure(distanceSolT, distanceSol, erreurHauteur, 1.0, 6, 0.0);
    }
    fIntervalleSonar = millis();    
  }
}
void restrictionMargesErreur()
{ // Pour chaque valeur, il l'envoie à erreurMesure() avec les données demandées
  erreurMesure((float)axT, ax, erreurAccelXY, sensibiliteAccel, 0, 0.0);
  erreurMesure((float)ayT, ay, erreurAccelXY, sensibiliteAccel, 1, 0.0);
  erreurMesure((float)azT, az, erreurAccelZ, sensibiliteAccel, 2, 1.0);
  erreurMesure((float)vxT, vx, erreurVitesse, sensibiliteVitesse, 3, 0.0);
  erreurMesure((float)vyT, vy, erreurVitesse, sensibiliteVitesse, 4, 0.0);
  erreurMesure((float)vzT, vz, erreurVitesse, sensibiliteVitesse, 5, 0.0);
}
//Fonction d'arrondissement des valeurs obtenues du MPU-6050
//Sert à maintenir les valeurs stables si elles ne changent pas beaucoup
void erreurMesure(float mesureT, float& mesure, float erreur, float sensibilite, int indexTab, float valReference)
{ 
  if(abs(mesureT/sensibilite-mesure) > erreur)//Si la valeur temporaire est plus grande que l'erreur, elle est assignée
  {                                           //à la "vraie" variable
    mesure = mesureT/sensibilite;
    tabCompteurs[indexTab] = 0;
  } 
  else if(tabCompteurs[indexTab] < nbLecturesCompteurs)//Le compteur de cette mesure augmente si la valeur est plus petite que l'erreur
  {
      tabCompteurs[indexTab]++;
  }
  else if(abs(valReference-mesureT/sensibilite) < erreur)//Si la valeur est suffisamment petite pendant un certain nombre de lectures (tabCompteurs[indexTab] == nbLecturesCompteurs),
  {                                                      //on lui accorde la valeur de référence, soit la valeur lorsque le MPU ne bouge pas.
    mesure = valReference;
    tabCompteurs[indexTab] = 0;
  }
}

void ecritureDonnee() //Écrit dans le port série les informations recueillies
{
    //Accélérations Angulaires
    Serial.print("ax");
    Serial.print(ax);
    Serial.println();
  
    Serial.print("ay");
    Serial.print(ay);
    Serial.println();
     
    Serial.print("az");
    Serial.print(az);
    Serial.println();
  
    
    //Vitesses angulaires    
    Serial.print("vx");
    Serial.print(vx);
    Serial.println();
  
    Serial.print("vy");
    Serial.print(vy);
    Serial.println();
  
    Serial.print("vz");
    Serial.print(vz);
    Serial.println();
  
    
    //Angles
    Serial.print("tx");
    Serial.print(euler[1]);
    Serial.println();
    
    Serial.print("ty");
    Serial.print(euler[0]);
    Serial.println();
  
    Serial.print("tz");
    Serial.print(euler[2]);
    Serial.println();    
    
    //Hauteur par rapport au sol
    Serial.print("dz");
    Serial.print(distanceSol);
    Serial.println();
}

void statutMPU()  
{ //Prends des info du MPU pour savoir s'il est prêt à donner des données
  //Si le MPU n'est pas interrompu et qu'il manque de place
  //dans le packet pour l'information lue
  while (!mpuInterrupt && nbByteFIFO < taillePacket) 
    {
      //  Attendre que le MPU ne soit plus interrompu ou qu'il y ait de la place disponible dans le packet
    }
     
    mpuInterrupt = false;   // Réinitialise le drapeau d'interruption
    iStatutMPU = senseurMPU.getIntStatus();  // Va chercher le statut du MPU en Byte
    nbByteFIFO = senseurMPU.getFIFOCount(); // Va chercher le compte FIFO
    
    if ((iStatutMPU & 0x10) || nbByteFIFO == 1024) 
    { // Vérifie s'il y a trop de données en liste (ne devrait pas arriver)        
        Serial.println(F("Débordement du FIFO (Overflow)!"));    
        senseurMPU.resetFIFO();  // Réinitialise les données pour recommencer
    } 
    else if (iStatutMPU & 0x02) 
    {//  Sinon, Regarder pour que les données DMP soient prêtes
        // (Devrait arriver plus régulièrement)
        while (nbByteFIFO < taillePacket) 
        { //Attendre que la longueur de données prêtes corresponde (devrait être très rapide)
          nbByteFIFO = senseurMPU.getFIFOCount();  
        }        
        senseurMPU.getFIFOBytes(fifoBuffer, taillePacket); // Lire un Packet venant du FIFO
        
        // Compte le nombre de FIFO pour lire plus, sans attendre l'interruption, 
        // s'il y a un packet ou plus de disponible
        nbByteFIFO -= taillePacket;  
    }
}

void statutDMP()
{  //Met le DMP en marche et regarde s'il est prêt à prendre et convertir les données
   if (iStatutDMP == 0) // Si le statut du DMP n'est pas une erreur.
   { 
        // Met le DMP en marche
        Serial.println(F("Mise en marche du DMP..."));
        senseurMPU.setDMPEnabled(true);

        //Mise en marche la détection d'interruption d'Arduino
        attachInterrupt(0, dmpDataReady, RISING);        

        // Le DMP est maintenant prêt
        Serial.println(F("DMP prêt!"));
        dmpReady = true;
        
        iStatutMPU = senseurMPU.getIntStatus();
        
        //Prend la grosseur du DMP packet pour une comparaison plus loin
        taillePacket = senseurMPU.dmpGetFIFOPacketSize();
    } 
    else 
    {
        // ERREUR!
        // 1 = Le chargement initial de la mémoire a échoué
        // 2 = La configuration du DMP a échoué
        // (Si tout arrête, le code devrait être 1)
        Serial.print(F("L'initialisation du DMP a échoué (code "));
        Serial.print(iStatutDMP);
        Serial.println(F(")"));
    }
}
