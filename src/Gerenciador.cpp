#include "Gerenciador.h"

#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <vector>

#include "Grafo.h"

void Gerenciador::comandos(std::unique_ptr<Grafo> grafo) {
    std::cout << "Digite uma das opcoes abaixo e pressione enter:" << std::endl
              << std::endl;
    std::cout << "(a) Fecho transitivo direto de um no;" << std::endl;
    std::cout << "(b) Fecho transitivo indireto de um no;" << std::endl;
    std::cout << "(c) Caminho minimo (Djikstra);" << std::endl;
    std::cout << "(d) Caminho minimo (Floyd);" << std::endl;
    std::cout << "(e) Arvore Geradora Minima (Algoritmo de Prim);" << std::endl;
    std::cout << "(f) Arvore Geradora Minima (Algoritmo de Kruskal);" << std::endl;
    std::cout << "(g) Arvore de caminhamento em profundidade;" << std::endl;
    std::cout << "(h) Raio, diametro, centro e periferia do grafo;" << std::endl;
    std::cout << "(0) Sair;" << std::endl
              << std::endl;

    char resp;
    std::cin >> resp;
    switch (resp) {
        case 'a':
            {
                char id_no = getIdEntrada();
                std::vector<char> fecho_transitivo_direto = grafo->fechoTransitivoDireto(id_no);

                if (fecho_transitivo_direto.size() > 0) {
                    std::cout << fecho_transitivo_direto[0];
                    for (int i = 1; i < fecho_transitivo_direto.size(); i++) {
                        std::cout << "," << fecho_transitivo_direto[i];
                    }
                }
                std::cout << "\n";

                if (perguntaImprimirArquivo("fecho_trans_dir.txt")) {
                    std::ofstream file_writer{"fecho_trans_dir.txt"};

                    if (fecho_transitivo_direto.size() > 0) {
                        file_writer << fecho_transitivo_direto[0];
                        for (int i = 1; i < fecho_transitivo_direto.size(); i++) {
                            file_writer << "," << fecho_transitivo_direto[i];
                        }
                    }
                    file_writer << "\n";

                    file_writer.close();
                }

                break;
            }
        case 'b':
            {
                char id_no = getIdEntrada();
                std::vector<char> fecho_transitivo_indireto = grafo->fechoTransitivoIndireto(id_no);

                if (fecho_transitivo_indireto.size() > 0) {
                    std::cout << fecho_transitivo_indireto[0];
                    for (int i = 1; i < fecho_transitivo_indireto.size(); i++) {
                        std::cout << "," << fecho_transitivo_indireto[i];
                    }
                }
                std::cout << "\n";

                if (perguntaImprimirArquivo("fecho_trans_indir.txt")) {
                    std::ofstream file_writer{"fecho_trans_indir.txt"};

                    if (fecho_transitivo_indireto.size() > 0) {
                        file_writer << fecho_transitivo_indireto[0];
                        for (int i = 1; i < fecho_transitivo_indireto.size(); i++) {
                            file_writer << "," << fecho_transitivo_indireto[i];
                        }
                    }
                    file_writer << "\n";

                    file_writer.close();
                }

                break;
            }
        case 'c':
            {
                char id_no_1 = getIdEntrada();
                char id_no_2 = getIdEntrada();
                std::vector<char> caminho_minimo_dijkstra = grafo->caminhoMinimoDijkstra(id_no_1, id_no_2);

                if (caminho_minimo_dijkstra.size() > 0) {
                    std::cout << caminho_minimo_dijkstra[0];
                    for (int i = 1; i < caminho_minimo_dijkstra.size(); i++) {
                        std::cout << "," << caminho_minimo_dijkstra[i];
                    }
                }
                std::cout << "\n";

                if (perguntaImprimirArquivo("caminho_minimo_dijkstra.txt")) {
                    std::ofstream file_writer{"caminho_minimo_dijkstra.txt"};

                    if (caminho_minimo_dijkstra.size() > 0) {
                        file_writer << caminho_minimo_dijkstra[0];
                        for (int i = 1; i < caminho_minimo_dijkstra.size(); i++) {
                            file_writer << "," << caminho_minimo_dijkstra[i];
                        }
                    }
                    file_writer << "\n";

                    file_writer.close();
                }

                break;
            }
        case 'd':
            {
                char id_no_1 = getIdEntrada();
                char id_no_2 = getIdEntrada();
                std::vector<char> caminho_minimo_floyd = grafo->caminhoMinimoFloyd(id_no_1, id_no_2);
                if (caminho_minimo_floyd.size() > 0) {
                    std::cout << caminho_minimo_floyd[0];
                    for (int i = 1; i < caminho_minimo_floyd.size(); i++) {
                        std::cout << "," << caminho_minimo_floyd[i];
                    }
                }
                std::cout << "\n";

                if (perguntaImprimirArquivo("caminho_minimo_floyd.txt")) {
                    std::ofstream file_writer{"caminho_minimo_floyd.txt"};

                    if (caminho_minimo_floyd.size() > 0) {
                        file_writer << caminho_minimo_floyd[0];
                        for (int i = 1; i < caminho_minimo_floyd.size(); i++) {
                            file_writer << "," << caminho_minimo_floyd[i];
                        }
                    }
                    file_writer << "\n";

                    file_writer.close();
                }

                break;
            }
        case 'e':
            {
                int tam;
                std::cout << "Digite o tamanho do subconjunto: ";
                std::cin >> tam;

                if (tam > 0 && tam <= grafo->getOrdem()) {
                    std::vector<char> ids = getConjuntoIds(*grafo, tam);
                    std::unique_ptr<Grafo> arvore_geradora_minima_prim = grafo->arvoreGeradoraMinimaPrim(ids);
                    std::cout << "Metodo de impressao em tela nao implementado" << std::endl
                              << std::endl;

                    if (perguntaImprimirArquivo("agm_prim.txt")) {
                        std::cout << "Metodo de impressao em arquivo nao implementado" << std::endl;
                    }

                } else {
                    std::cout << "Valor invalido" << std::endl;
                }

                break;
            }
        case 'f':
            {
                int tam;
                std::cout << "Digite o tamanho do subconjunto: ";
                std::cin >> tam;

                if (tam > 0 && tam <= grafo->getOrdem()) {
                    std::vector<char> ids = getConjuntoIds(*grafo, tam);
                    std::unique_ptr<Grafo> arvore_geradora_minima_kruskal = grafo->arvoreGeradoraMinimaKruskal(ids);

                    arvore_geradora_minima_kruskal->printAdjacencyList();

                    if (perguntaImprimirArquivo("agm_kruskal.txt")) {
                        std::ofstream file_writer{"agm_kruskal.txt"};

                        file_writer << arvore_geradora_minima_kruskal->formattedAdjacencyList();

                        file_writer.close();
                    }
                } else {
                    std::cout << "Valor invalido" << std::endl;
                }

                break;
            }
        case 'g':
            {
                char id_no = getIdEntrada();
                std::unique_ptr<Grafo> arvore_caminhamento_profundidade = grafo->arvoreCaminhamentoProfundidade(id_no);

                arvore_caminhamento_profundidade->printAdjacencyList();

                if (perguntaImprimirArquivo("arvore_caminhamento_profundidade.txt")) {
                    std::ofstream file_writer{"arvore_caminhamento_profundidade.txt"};

                    file_writer << arvore_caminhamento_profundidade->formattedAdjacencyList();

                    file_writer.close();
                }

                arvore_caminhamento_profundidade->printGraph();

                break;
            }
        case 'h':
            {
                auto eccentricities = grafo->getEccentricities();
                int raio = grafo->raio(eccentricities);
                int diametro = grafo->diametro(eccentricities);
                std::vector<char> centro = grafo->centro(eccentricities);
                std::vector<char> periferia = grafo->periferia(eccentricities);

                std::cout << raio << "\n"
                          << diametro << "\n";

                if (centro.size() > 0) {
                    std::cout << centro[0];
                    for (int i = 1; i < centro.size(); i++) {
                        std::cout << "," << centro[i];
                    }
                }
                std::cout << "\n";

                if (periferia.size() > 0) {
                    std::cout << periferia[0];
                    for (int i = 1; i < periferia.size(); i++) {
                        std::cout << "," << periferia[i];
                    }
                }
                std::cout << "\n";

                if (perguntaImprimirArquivo("raio_diametro_centro_periferia.txt")) {
                    std::ofstream file_writer{"raio_diametro_centro_periferia.txt"};

                    file_writer << raio << "\n"
                                << diametro << "\n";

                    if (centro.size() > 0) {
                        file_writer << centro[0];
                        for (int i = 1; i < centro.size(); i++) {
                            file_writer << "," << centro[i];
                        }
                    }
                    file_writer << "\n";

                    if (periferia.size() > 0) {
                        file_writer << periferia[0];
                        for (int i = 1; i < periferia.size(); i++) {
                            file_writer << "," << periferia[i];
                        }
                    }
                    file_writer << "\n";

                    file_writer.close();
                }

                break;
            }
        case '0':
            {
                exit(0);
            }
        default:
            {
                std::cout << "Opção inválida" << std::endl;
            }
    }

    comandos(std::move(grafo));
}

char Gerenciador::getIdEntrada() {
    std::cout << "Digite o id de um no: ";
    char id;
    std::cin >> id;
    std::cout << std::endl;
    return id;
}

std::vector<char> Gerenciador::getConjuntoIds(const Grafo& grafo, int tam) {
    std::vector<char> ids = {};
    while ((int) ids.size() < tam) {
        char id_no = getIdEntrada();
        bool existe = false;
        for (auto& no : grafo.getListaDeAdjacencia()) {
            if (no.first == id_no) {
                existe = true;
                break;
            }
        }

        if (!existe) {
            std::cout << "Vertice nao existe" << std::endl;
        } else {
            bool repetido = find(ids.begin(), ids.end(), id_no) != ids.end();
            if (repetido) {
                std::cout << "Valor repetido" << std::endl;
            } else {
                ids.push_back(id_no);
            }
        }
    }

    return ids;
}

bool Gerenciador::perguntaImprimirArquivo(std::string nome_arquivo) {
    std::cout << "Imprimir em arquivo externo? (" << nome_arquivo << ")" << std::endl;
    std::cout << "(1) Sim;" << std::endl;
    std::cout << "(2) Nao." << std::endl;
    int resp;
    std::cin >> resp;
    std::cout << std::endl;

    switch (resp) {
        case 1:
            return true;
        case 2:
            return false;
        default:
            std::cout << "Resposta invalida" << std::endl;
            return perguntaImprimirArquivo(nome_arquivo);
    }
}

// receives the name of the file to be read as it's only parameters,
// returns a null unique_ptr if the provided filename doesn't point to a valid file,
// otherwise returns an unique_ptr to a new graph initialized with the parameters given in that file
std::unique_ptr<Grafo> Gerenciador::lerArquivo(const std::string& nome_arquivo) {
    std::ifstream file_reader{nome_arquivo};

    if (!file_reader.is_open()) {
        file_reader.close();
        return std::unique_ptr<Grafo>{};
    }

    std::string header;
    std::getline(file_reader, header);

    std::array<bool, 3> header_data;

    for (int i = 0; i < 3; i++) {
        int delimiter_position = header.find(" ");

        std::string token = header.substr(0, delimiter_position);
        header_data[i] = token == "1" ? true : false;

        header.erase(0, delimiter_position + 1);  // +1 to also remove the delimiter
    }

    auto graph = std::unique_ptr<Grafo>{new Grafo{header_data[0], header_data[1], header_data[2]}};

    std::string node_amount_string;
    std::getline(file_reader, node_amount_string);
    int node_amount;
    try {
        node_amount = std::stoi(node_amount_string);
    } catch (const std::exception& exception) {
        return std::unique_ptr<Grafo>{};
    }

    for (int i = 0; i < node_amount; i++) {
        std::string node_information;
        std::getline(file_reader, node_information);

        if (!header_data[2]) {                     // different operations in case of a graph with weighted node
            if (node_information.length() != 1) {  // the node_id can only be made-up of one char
                return std::unique_ptr<Grafo>{};
            }

            graph->createNode(node_information[0]);
        } else {
            int delimiter_position = node_information.find(" ");

            std::string node_id = node_information.substr(0, delimiter_position);
            node_information.erase(0, delimiter_position + 1);  // +1 to also remove the delimiter

            if (node_id.length() != 1) {  // the node_id can only be made-up of one char
                return std::unique_ptr<Grafo>{};
            }

            try {
                graph->createNode(node_id[0], std::stoi(node_information));  // after erasure, the only information left in node_information should be the weight
            } catch (const std::exception& exception) {
                return std::unique_ptr<Grafo>{};
            }
        }
    }

    std::string edge_information;
    while (std::getline(file_reader, edge_information)) {  // will loop until the end of the file
        int starting_node_delimiter_position = edge_information.find(" ");
        std::string starting_node = edge_information.substr(0, starting_node_delimiter_position);
        edge_information.erase(0, starting_node_delimiter_position + 1);  // +1 to also remove the delimiter

        if (starting_node.length() != 1) {  // the node_id can only be made-up of one char
            return std::unique_ptr<Grafo>{};
        }

        if (!header_data[1]) {                     // different operations in case of a graph with weighted edges
            if (edge_information.length() != 1) {  // the node_id can only be made-up of one char
                return std::unique_ptr<Grafo>{};
            }

            graph->createEdge(starting_node[0], edge_information[0]);  // after erasure, the only information left in edge_information should be the ending_node
        } else {
            int ending_node_delimiter_position = edge_information.find(" ");
            std::string ending_node = edge_information.substr(0, ending_node_delimiter_position);
            edge_information.erase(0, ending_node_delimiter_position + 1);  // +1 to also remove the delimiter

            if (ending_node.length() != 1) {  // the node_id can only be made-up of one char
                return std::unique_ptr<Grafo>{};
            }

            try {
                graph->createEdge(starting_node[0], ending_node[0], std::stoi(edge_information));  // after erasure, the only information left in edge_information should be the weight
            } catch (const std::exception& exception) {
                return std::unique_ptr<Grafo>{};
            }
        }
    }

    file_reader.close();
    return std::move(graph);
}
