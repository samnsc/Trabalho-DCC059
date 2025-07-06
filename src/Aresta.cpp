#include "Aresta.h"

// a delegating constructor is used here because const class members have to be
// instantiated before the initializer
Aresta::Aresta(char id_no_alvo, int peso): ID_NO_ALVO(id_no_alvo), PESO(peso) {}

char Aresta::getIdNoAlvo() const {
    return this->ID_NO_ALVO;
}

int Aresta::getPeso() const {
    return this->PESO;
}
