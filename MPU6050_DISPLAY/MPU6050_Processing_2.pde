import processing.serial.*;   //  Donne accès au port série
//Télécharger préalablement Toxiclibs
import toxi.geom.*;  //Pour les Quaternions
//Télécharger préalablement OSCP5
import netP5.*;  //Pour UDP

Serial portSerie;  // Crée un objet de la classe Serial

final int portUDP = 8080;//Port utilisé pour la connexion UDP
final String gatewayIp = "172.16.10.1";  //Adresse IP du drone pour la connexion UDP
//--------------------------------------------------------------------------------------------------------------------------
//Constantes et variables pour l'interprétation des donnés provenant du Arduino
final short nbLecturesCompteurs = 5;
final float erreurAngle = 0.1f; //Erreur d'arrondissement pour les angles d'Euler en X et Z
final float erreurRotation = 0.5f; //Erreur d'arrondissement pour l'angle d'Euler en Y
final float limiteAngle = PI / 2.0; //Angle maximum en X et Z (90°)
final float limiteRotation = PI / 4.0; //Angle maximum en Y (45°)
final float angleMaxProfondeurPositive = 1.06f;//Angles maximums acceptables pour le changement d'accélération de la rotation de la main en Z (avant-arriere) 
final float angleMaxProfondeurNegative= -1.04f;//*Déterminés expérimentalement
//--------------------------------------------------------------------------------------------------------------------------
//Constantes et variables pour l'utilisation des donnés provenant du Arduino
boolean infoPause = false;
boolean doitVoler = false;
boolean differenteValLaterale = true;
boolean differenteValProfondeur = true;
boolean differenteValRotation = true;
boolean hauteurInitialise = false;  //Dit si on a déjà déclaré le milieuHauteur ou non
short[] tabCompteurs = new short[3];
float fAx, fAy, fAz;   //Accélérations mesurées en x, y et z
float fVx, fVy, fVz;   //Vitesses angulaires mesurées en x, y et z
float roulementX = 0, rotationY = 0, profondeurZ = 0;   //Angles
float fDistanceSol; //Distance entre le gant et le sol
float fAxI, fAyI, fAzI;   //Accélérations mesurées en x, y et z pour les infos à l'écran
float fVxI, fVyI, fVzI;   //Vitesses angulaires mesurées en x, y et z pour les infos à l'écran
float roulementXI, rotationYI, profondeurZI;   //Angles Euler pour les infos à l'écran
float distanceSolI;       //Distance du sol pour les infos à l'écran
float roulementXT = 0, rotationYT = 0, profondeurZT = 0;   //Angles Euler temporaires
float milieuHauteur = 0.0f;  //La hauteur par rapport au sol du milieu - soit la hauteur de départ
float limiteHauteur = 50.0f;
float fIntervalleInfo = 0;
//--------------------------------------------------------------------------------------------------------------------------
//Valeurs de lecture provenant du port série
String sIndiceValeur;  // Données reçues du port série
String sAx, sAy, sAz;   //Accélérations mesurées en x, y et z
String sVx, sVy, sVz;   //Vitesses angulaires mesurées en x, y et z
String sAngleX, sAngleY, sAngleZ;   //Angles
String sDistanceSol; //Distance entre le gant et le sol

PFont police;  //Police de caractère du texte de l'interface (Pour les infos)
PImage imageDessusMPU, imageDessousMPU, imageAvantArriereMPU, imageCotesMPU;  //Images du MPU

Quaternion quat1 = new Quaternion(1, 0, 0, 0);
UdpClient udpClient2;

//Indexes des commandes selon le tableau parametres
//Remplacement d'une énumération, apparemment pas supportée par processing
static abstract class PARAMETRES_INDEXES {
  static final short HEADER_UDP_0 = 0; //Première partie de l'entête UDP
  static final short HEADER_UDP_1 = 1; //Deuxième partie de l'entête UDP
  static final short LIFT = 2; //Hauteur
  static final short TURN = 3; //Rotation
  static final short ADVANCE = 4; //Mouvement longitudinal (avant-arière)
  static final short STRAFE = 5; //Mouvement latéral
  static final short YAW = 6; //Valeur d'équilibre pour la rotation 
  static final short PITCH = 7; //Valeur d'équilibre pour le mouvement longitudinal
  static final short ROLL = 8; //Valeur d'équilibre pour le mouvement latéral
  static final short THROTTLE = 9; //Valeur d'équilibre pour l'accélération verticale
  static final short CHECK_SUM = 10; //Somme de contrôle / check sum : Sert à s'assurer que le paquet de bytes à le bon nombre (doit être pair pour fonctionner)
                                     //Une valeur impaire ferait changer tous les bytes à zéro
}
//Ensemble de valeurs pour les changements des quantités de commandes
static abstract class changementCommandes {
  static final byte VAL_GAUCHE = 0x39;
  static final byte VAL_DROITE = 0x3f;
  static final byte DIFFERENCE_LATERALE = 0x40;
  static final byte DIFFERENCE_ROTATION = 0x40;
  static final byte DIFFERENCE_PROFONDEUR = 0x46;
  static final byte DIFFERENCE_HAUTEUR = 0x60;
}
//Tableau de commandes par défaut (drone qui fait du surplace)
byte[] parametresDefaut = {
  (byte)0xff, 
  0x04, 
  (byte)0x9f,
  0x3f,
  (byte)0xb9,
  0x39,
  (byte)0x90,
  0x12,
  0x0b,
  0x5f,
  0x00,
};
////Tableau d'arrêt, rempli de 0
byte[] parametresArret = {
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00, 
  0x00,
};

byte[] parametres;//Tableau envoyé lors de la boucle de vol

void setup()
{
  //frameRate(500);
  parametres = new byte[11];

  for (int i = 0; i < parametres.length; i++)//Initialisation du tableau envoyé au tableau par défaut (pour le démarrage)
  {
    parametres[i] = parametresDefaut[i];
  }

  udpClient2 = new UdpClient(gatewayIp, portUDP);
  parametres[PARAMETRES_INDEXES.CHECK_SUM] = checkSum();

  //size(Largeur de l'écran, Hauteur de l'écran, rendu de l'interface (3D dans notre cas))
  size(800, 600, P3D);
  textureMode(NORMAL);  // Ne pas changer NORMAL pour IMAGE(défaut), 
                        // les images png utilisées ne fonctionnent pas sinon.
  fill(255);  //Clareté du blanc dans la police (0 à 255) (255 = max)
  noStroke();  //Ne pas avoir de contour sur le MPU de l'interface

  // initialise les lumières et l'effet d'anticrénelage (lissage de police)
  lights();
  smooth();

  // Mettre la police de caractère désirée (Doit être dans le fichier Data)
  // une police peut être acquise en haut du programme dans Outils -> Générer la police...
  police = loadFont("AngsanaNew-Bold-48.vlw"); 

  // Chargement des textures du MPU (prisme)
  // Les fichiers PNG nous donne la possibilité d'avoir des trous dans l'image, ce qui augmente le réalisme.
  imageDessusMPU = loadImage("MPU6050 A.png");   //Dessus
  imageDessousMPU = loadImage("MPU6050 B.png");  //Dessous
  imageAvantArriereMPU = loadImage("MPU6050 F.png");   //Devant
  imageCotesMPU = loadImage("MPU6050 E.png");     //Côté

  portSerie = new Serial(this, Serial.list()[0], 115200); // mettre le bon nombre de baud comme 3e donnée
}
//Boucle de vol
void draw() 
{
  if (doitVoler)//Si la touche enter a été appuyée, le drone doit voler
  {
    inputMPU(); //Appel de la fonction de changement des mesures pour la quantite à envoyer au drone
    parametres[PARAMETRES_INDEXES.CHECK_SUM] = checkSum(); //Asssignation de la valeur du dernier élément du tableau de commandes
    udpClient2.send(parametres); //Envoi du tableau de commandes au drone
  } else
  {
    udpClient2.send(parametresArret); //Si la touche delete a été appuyée ou que doitVoler n'a jamais été changé à true
  } 
  
  //Couleur du fond d'écran
  background(0);  //Noir dans notre cas

  drawInfo();
  drawCube();
}

void drawInfo()
{
  //Met l'actualisation des infos moins rapides afin de mieux apercevoir les données.
  //Si le temps d'intervalle est écoulé et que les informations ne sont pas sur pause
  if((millis() - fIntervalleInfo >= 100 || millis() < 100) && !infoPause)
  { //Actualisation des données
    fVxI = fVx; fVyI = fVy; fVzI = fVz;
    fAxI = fAx; fAyI = fAy; fAzI = fAz;
    roulementXI = roulementXT; rotationYI = rotationYT; profondeurZI = profondeurZT;
    distanceSolI = fDistanceSol;
    fIntervalleInfo = millis();  //Remet le temps écoulé à 0
  }
  textFont(police, 20);  //Données de la police de caractère
  textAlign(LEFT, TOP);  //Aligné le texte pour qu'il commence en haut à gauche
  //Écriture des informations
  text(" Angle Euler x (roulement) = " + roulementXI + "\n Angle Euler y (rotation) = " 
       + rotationYI + "\n Angle Euler z (profondeur) = " + profondeurZI +
       "\n\n Vx = " + fVxI + "\n Vy = " + fVyI + "\n Vz = " + fVzI +
       "\n\n Ax = " + fAxI + "\n Ay = " + fAyI + "\n Az = " + fAzI +
       "\n\n Distance du sol (cm) = " + distanceSolI, 20, 20);
}

void drawCube() 
{    
  //PushMatrix() doit être utilisé avec PopMatrix()
  //Écrire un code entre ces deux fonctions bouge les images sans bouger le reste
  //Pour notre code, ce n'est pas vraiment nécessaire, puisqu'on n'a pas d'autres images dans l'interface
  //pushMatrix();  //Enregistre le point d'origine du plan
  
  translate(width/2, height/2 + 50, 0);  //Mettre le MPU à l'endroit souhaité au départ à l'écran
  scale(10);  //Grosseur du MPU à l'écran
  
  //Création du quaternion avec les angles de Euler afin de pouvoir avoir les angles d'axe
  quat1 = Quaternion.createFromEuler(-roulementXT, -rotationYT, profondeurZT); 

  float[] axis = quat1.toAxisAngle();  //Stocke les angles d'axe
  rotate(axis[0], axis[1], axis[3], axis[2]);  //Rotation du cube avec les angles d'axe
  rotateY(-PI/2);  //Rotation afin d'ajuster pour avoir la bonne direction de départ

  drawDessus(imageDessusMPU);  
  drawDessous(imageDessousMPU);
  drawCotes(imageCotesMPU);
  drawAvantArriere(imageAvantArriereMPU);

  //popMatrix();  //Remet le point d'origine comme avant pushMatrix()
}

void serialEvent(Serial port)
{
  //intervalle = millis();
  int iValeurLue = 0; //Valeur lue dans le port série
  //if (port.available() > 0)
  while (port.available() > 0) // tant que les données sont disponibles dans le port
  {
    iValeurLue = port.read();  //Lire un int dans le port série

    if (estAVTD(iValeurLue)) //Si la valeur = a, v, t ou d
    {
      stockageIndiceAVTD(iValeurLue);
    }    
    if (estXYZ(iValeurLue) && sIndiceValeur != null)  //Si la valeur = x, y ou z
    {      
      stockageIndiceXYZ(iValeurLue);
    }
    if (estChiffreOuPoint(iValeurLue) && sIndiceValeur != null) //Si la valeur = un chiffre ou un point
    {
      stockageValeur(iValeurLue);
    }
  }
}

void keyPressed()
{
  if (key == ENTER)// Si la touche enter est appuyée
  {
    for (int i = 0; i < parametres.length; i++)//Initialisation du tableau envoyé au tableau par défaut (pour le démarrage)
  {
    parametres[i] = parametresDefaut[i];
  }
    doitVoler = true;
    hauteurInitialise = false;  //Réinitialiser la hauteur milieu lorsque la touche enter est appuyée
    println("Depart");
  } else if (key == DELETE || key == BACKSPACE)// Si la touche delete est appuyée
  {
    doitVoler = false;
    println("Arret");
  }
  if (key == ' ')  // Si la touche espace est appuyée
  {
    //Met les informations de l'interface sur pause ou play
    infoPause = !infoPause;
  }
}
//Fonction de calcul de la somme de contrôle:
//La somme de contrôle est calculée.
//On calcule ensuite si la somme de tous les bytes envoyés est binairement paire ou impaire.
byte checkSum() 
{
  int sum = 0, i;
  for (i = 1; i <= 9; i++) 
  {
    sum += parametres[i];
  }
  return (byte)(parametresDefaut[PARAMETRES_INDEXES.HEADER_UDP_0] - (sum % 256));
}
//Fonctionnement assemblant les apppels de fonction de changement de quantité pour chaque angle d'Euler
void inputMPU()
{
  changerQuantite("lateral", erreurMesure(roulementXT, roulementX, erreurAngle, 0, 0.0), limiteAngle, PARAMETRES_INDEXES.STRAFE, parametresDefaut[PARAMETRES_INDEXES.STRAFE], changementCommandes.VAL_DROITE, differenteValLaterale);
  //changerQuantite("rotation", -erreurMesure(rotationYT, rotationY, erreurRotation, 1, 0.0), limiteRotation, PARAMETRES_INDEXES.TURN, parametresDefaut[PARAMETRES_INDEXES.TURN], changementCommandes.DIFFERENCE_ROTATION, differenteValRotation);
  changerQuantite("profondeur", erreurMesure(profondeurZT, profondeurZ, erreurAngle, 2, 0.0), limiteAngle * 2, PARAMETRES_INDEXES.ADVANCE, parametresDefaut[PARAMETRES_INDEXES.ADVANCE], changementCommandes.DIFFERENCE_PROFONDEUR, differenteValProfondeur);
  changerHauteur();
}
// Fonction qui prend la distance par rapport au sol et la compare avec celle de départ
// pour savoir si elle est plus basse ou plus haute et, ainsi, changer la valeur de lift du drone
void changerHauteur()
{
  if(hauteurInitialise == false)
  {    
    milieuHauteur = fDistanceSol;
    hauteurInitialise = true;
  }
  else if(roulementXT <= 0.4f && roulementXT >= -0.4f && profondeurZT <= 0.4f && profondeurZT >= -0.4f)
  { //Variation du LIFT proportionnellement à la hauteur du gant
    parametres[PARAMETRES_INDEXES.LIFT] = byte(abs((1 -((milieuHauteur - fDistanceSol)* 3f/milieuHauteur)) * parametresDefaut[PARAMETRES_INDEXES.LIFT]));
  }
}

//Fonction qui interprète les changements d'angles pour les convertir en % des quantités envoyées vers le drone
//nom: chaine de caractères pour identifier l'angle
//angle: l'angle de rotation utilisé
//limite: valeur maximale pour l'angle de rotation présent utilisé (utilisée pour la variation inversement proportionnelle à l'angle)
//index: index pour la position dans le tableau de paramètres
//milieu: quantité stable (par défaut) utilisée comme valeur de référence
//difference: valeur exprimant la différence entre la quantité maximale et la quantité milieu (utilisée pour la variation proportionnelle à l'angle)
//differenteVal: condition exprimant si la valeur est différente ou pas (utilisée pour savoir si on met la valeur par défaut ou non)
void changerQuantite(String nom, float angle, float limite, int index, byte milieu, byte difference, boolean differenteVal)
{
  if (angle > 0.1 && angle < limite && parametres[index] != byte(abs((1 -(angle/limiteAngle)) * milieu)))//Rotation à gauche, mouvement latéral à gauche ou mouvement longitudinal à l'arrière
  {
    if (nom =="profondeur")
    {
      angle*=3;//augmentation du % de vitesse vers l'avant dû au mouvement limité du poignet
      if (angle > 3 * angleMaxProfondeurPositive)
      {
        parametres[index] = 0;
      }else
         parametres[index] = byte(abs((1 -(angle/limite)) * milieu));
    }else 
    parametres[index] = byte(abs((1 -(angle/limite)) * milieu));
    differenteVal = true;
    //println("1" + nom + " " + parametres[index]);
  } else if (angle < 0.1 && angle > -limite && parametres[index] != byte(abs(angle/limiteAngle) * difference + milieu))//Rotation à droite, mouvement latéral à droite ou mouvement longitudinal à l'avant
  {
    if (nom =="profondeur")
    {
      angle*=3;//augmentation du % de vitesse vers l'arrière dû au mouvement limité du poignet
      if (angle < 3 * angleMaxProfondeurNegative)
      {
        parametres[index] = byte(-1);
      }else
        parametres[index] = byte(abs(angle/limite) * difference + milieu);
    }else 
      parametres[index] = byte(abs(angle/limite) * difference + milieu);
    differenteVal = true;
    //println("2" + nom + " " + parametres[index]);
  } else if (abs(angle) < 0.25f && differenteVal) //Rotation nulle, mouvement latéral nul ou mouvement longitudinal nul si l'anngle de la direction est 
  {
    parametres[index] = parametresDefaut[index];
    differenteVal = false;
    //println("3" + nom + " " + parametres[index]);
  }
  
  //Changement du bool approprié régissant si une direction doit être changée ou non
  if (nom == "lateral")
  {
    differenteValLaterale = differenteVal;
  } else if (nom == "profondeur")
  {
    differenteValProfondeur = differenteVal;
  } else if (nom == "rotation")
  {
    differenteValRotation = differenteVal;
  }
}
//Même fonction 
float erreurMesure(float mesureT, float mesure, float erreur, int indexTab, float valReference)
{
  if (abs(mesureT-mesure) > erreur)
  {
    mesure = mesureT;
    tabCompteurs[indexTab] = 0;
  } else if (tabCompteurs[indexTab] < nbLecturesCompteurs)
  {
    tabCompteurs[indexTab]++;
  } else if (abs(valReference-mesureT) < erreur)  
  {
    mesure = valReference;
    tabCompteurs[indexTab] = 0;
  }
  return mesure;
}

void drawDessus(PImage image1) 
{
  beginShape(QUADS); //Commence à dessiner la forme d'un quadrilatère
  texture(image1);  //Dessine l'image dans le quadrilatère
  // Prend les coordonnées du quadrilatère, (x,y,z,u,v).
  // u et v sont les coordonnées horizontales et verticales 
  // pour le mappage de texture.
  vertex(-20, -1, -15, 0, 0);
  vertex( 20, -1, -15, 1, 0);
  vertex( 20, -1, 15, 1, 1);
  vertex(-20, -1, 15, 0, 1);

  endShape();
}

void drawDessous(PImage image1) 
{
  beginShape(QUADS); //Dessine la forme d'un quadrilatère
  texture(image1);  //Dessine l'image dans le quadrilatère
  // Prend les côtés du quadrilatère, (x,y,z,u,v).
  // u et v sont les coordonnées horizontales et verticales 
  // pour le mappage de texture.
  vertex(-20, 1, 15, 0, 0);
  vertex( 20, 1, 15, 1, 0);
  vertex( 20, 1, -15, 1, 1);
  vertex(-20, 1, -15, 0, 1);

  endShape();
}

void drawCotes(PImage image1) 
{
  beginShape(QUADS); //Dessine la forme d'un quadrilatère
  texture(image1);  //Dessine l'image dans le quadrilatère
  // Prend les côtés du quadrilatère, (x,y,z,u,v).
  // u et v sont les coordonnées horizontales et verticales 
  // pour le mappage de texture.
  // Côté gauche :
  vertex(-20, -1, 15, 0, 0);
  vertex( 20, -1, 15, 1, 0);
  vertex( 20, 1, 15, 1, 1);
  vertex(-20, 1, 15, 0, 1);

  // Côté droit :
  vertex( 20, -1, -15, 0, 0);
  vertex(-20, -1, -15, 1, 0);
  vertex(-20, 1, -15, 1, 1);
  vertex( 20, 1, -15, 0, 1);

  endShape();
}

void drawAvantArriere(PImage image1) 
{
  beginShape(QUADS); //Dessine la forme d'un quadrilatère
  texture(image1);  //Dessine l'image dans le quadrilatère
  // Prend les côtés du quadrilatère, (x,y,z,u,v).
  // u et v sont les coordonnées horizontales et verticales 
  // pour le mappage de texture.
  // Arrière :
  vertex( 20, -1, 15, 0, 0);
  vertex( 20, -1, -15, 1, 0);
  vertex( 20, 1, -15, 1, 1);
  vertex( 20, 1, 15, 0, 1);

  // Devant :
  vertex(-20, -1, -15, 0, 0);
  vertex(-20, -1, 15, 1, 0);
  vertex(-20, 1, 15, 1, 1);
  vertex(-20, 1, -15, 0, 1);

  endShape();
}

// La valeur intéressée est en int et on regarde dans la table ASCII pour voir si c'est les données qu'on recherche.
// L'indice de la valeur nous indique si cette valeur est pour une vitesse, accélération ou angle. Sinon, on ne la considère pas.
boolean estChiffreOuPoint(int iValeur)
{ //Si la valeur dans la table ASCII correspond à un nombre, un point ou un signe négatif
  if ((iValeur >= 48 && iValeur <= 57) || iValeur == 46 || iValeur == 45) 
  {
    return true;
  } else
  {
    return false;
  }
}

boolean estAVTD(int iValeur)
{ // 97 = a, 118 = v, 116 = t, 100 = d dans la table ASCII
  if (iValeur == 97 || iValeur == 118 || iValeur == 116 || iValeur == 100)
  {
    return true;
  } else
  {
    return false;
  }
}

boolean estXYZ(int iValeur)
{
  if (iValeur >= 120 && iValeur <= 122) //Si la valeur dans la table ASCII correspond à x,y ou z
  {
    return true;
  } else
  {
    return false;
  }
}


void stockageValeur(int iValeur)
{
  if (sIndiceValeur.equals("ax"))
  {
    if (sAx == null)
    {
      sAx = Character.toString(char(iValeur));
    } else
    {
      sAx += Character.toString(char(iValeur));
    }
  } else if (sIndiceValeur.equals("ay"))
  {
    if (sAy == null)
    {
      sAy = Character.toString(char(iValeur));
    } else
    {
      sAy += Character.toString(char(iValeur));
    }
  } else if (sIndiceValeur.equals("az"))
  {
    if (sAz == null)
    {
      sAz = Character.toString(char(iValeur));
    } else
    {
      sAz += Character.toString(char(iValeur));
    }
  } else if (sIndiceValeur.equals("vx"))
  {
    if (sVx == null)
    {
      sVx = Character.toString(char(iValeur));
    } else
    {
      sVx += Character.toString(char(iValeur));
    }
  } else if (sIndiceValeur.equals("vy"))
  {
    if (sVy == null)
    {
      sVy = Character.toString(char(iValeur));
    } else
    {
      sVy += Character.toString(char(iValeur));
    }
  } else if (sIndiceValeur.equals("vz"))
  {
    if (sVz == null)
    {
      sVz = Character.toString(char(iValeur));
    } else
    {
      sVz += Character.toString(char(iValeur));
    }
  } else if (sIndiceValeur.equals("tx"))
  {
    if (sAngleX == null)
    {
      sAngleX = Character.toString(char(iValeur));
    } else
    {
      sAngleX += Character.toString(char(iValeur));
    }
  } else if (sIndiceValeur.equals("ty"))
  {
    if (sAngleY == null)
    {
      sAngleY = Character.toString(char(iValeur));
    } else
    {
      sAngleY += Character.toString(char(iValeur));
    }
  } else if (sIndiceValeur.equals("tz"))
  {
    if (sAngleZ == null)
    {
      sAngleZ = Character.toString(char(iValeur));
    } else
    {
      sAngleZ += Character.toString(char(iValeur));
    }
  }
  else if (sIndiceValeur.equals("dz"))
  {
    if (sDistanceSol == null)
    {
      sDistanceSol = Character.toString(char(iValeur));
    } else
    {
      sDistanceSol += Character.toString(char(iValeur));
    }
  }
}

void stockageIndiceAVTD(int iValeur)
{
  if (sIndiceValeur != null)
  {
    if (sIndiceValeur.equals("ax") && sAx != null)
    {
      fAx = float(sAx);
      sAx = null;
    } else if (sIndiceValeur.equals("ay") && sAy != null)
    {
      fAy = float(sAy);   
      sAy = null;
    } else if (sIndiceValeur.equals("az") && sAz != null)
    {
      fAz = float(sAz);     
      sAz = null;
    } else if (sIndiceValeur.equals("vx") && sVx != null)
    {
      fVx = float(sVx);
      sVx = null;
    } else if (sIndiceValeur.equals("vy") && sVy != null)
    {
      fVy = float(sVy);      
      sVy = null;
    } else if (sIndiceValeur.equals("vz") && sVz != null)
    {
      fVz = float(sVz);     
      sVz = null;
    } else if (sIndiceValeur.equals("tx") && sAngleX != null)
    {
      roulementXT = float(sAngleX);
      sAngleX = null;
    } else if (sIndiceValeur.equals("ty") && sAngleY != null)
    {
      rotationYT = float(sAngleY);   
      sAngleY = null;
    } else if (sIndiceValeur.equals("tz") && sAngleZ != null)
    {
      profondeurZT = float(sAngleZ);   
      sAngleZ = null;
    }
    else if (sIndiceValeur.equals("dz") && sDistanceSol != null)
    {
      fDistanceSol = float(sDistanceSol);   
      sDistanceSol = null;
    }
  }
  sIndiceValeur = Character.toString(char(iValeur));
}

void stockageIndiceXYZ(int iValeur)
{
  if (sIndiceValeur.equals("a") || sIndiceValeur.equals("v") || sIndiceValeur.equals("t") || sIndiceValeur.equals("d"))
  {        
    sIndiceValeur += Character.toString(char(iValeur));
  }
} 
