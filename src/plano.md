# Problemas 
1. FeatureStruct alterar struct de objeto para ter keypoints - done
2. MapLoader prepara-lo para extrair keypoints -done
3. fake detetor a mandar quadrados ou outros objetos (centro + orientação mais tipo) (miguel)
4. no pf, preparar 2 funções no measurement - 1 para coner (já está mas meter em função que é chamada se for corner) 1 para objetos ( que de acordo com o tipo reconstroi o bjeto visto pelo robot, faz feature matching com esse tipo de objetos no mapa, e depois de ter o objeto desse tipo mais proximo calcula a likelihood)