/*!
 *
 */



#define CURRENT_TABLE_SIZE 64


class CurrentSensor
{
public:

  /*!
   * Constructeur
   *
   *  initialisation des variables et des drapeaux
   *  calculs des seuils haut et bas en entier (pour reduire le temps de calculs
   */
  CurrentSensor (float toVolts, float offset, float sensitivity,  float fSeuil);

  /*!
   * Initialisation des valeurs brutes pour le filtrage filtre
   */
  void initialisation (void);

  /*!
   * Affectation d'une nouvelle valeur dans le tableau de valeurs brutes
   * et gestion de l'index de la table
   */
  void setNewValue (uint16_t v);



  /*!
   *
   *
   */
  bool isOverCurrentDetected (void);

  /*!
   *
   */
 float getValue (void);

 /*!
  *
  */
void periodicManagement (void);

private:
 /*!
  * Calcul du filtre ou de la moyenne et detection de surintensité
  *
  */
  void filterRawValues (void);


  uint16_t rawCurrentValues[CURRENT_TABLE_SIZE];
  uint16_t rawCurrentTableIndex;
  uint16_t averageRawCurrent;
  bool overCurrent;

  float filteredCurrent;
  float toVolts_;
  float offset_;
  float sensitivity_;
  uint16_t iSeuilPos_;
  uint16_t iSeuilNeg_;
  volatile bool flagNewVal;

};

