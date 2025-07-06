#ifndef __ARESTA_H__
#define __ARESTA_H__

class Aresta {
    public:
        Aresta(char id_no_alvo, int peso);

        char getIdNoAlvo() const;
        int getPeso() const;

    private:
        const char ID_NO_ALVO;
        const int PESO;
};

#endif
