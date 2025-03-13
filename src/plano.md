# Problemas 
1. Partículas saem fora do mapa
2. Ruído pode fazer com que pesos fiquem negativos
3. Odometria a rodar diverge demasiado
4. Mudar a lógica do motionUpdate para utilizar deltas e não os p.init's
5. Ângulos são algumas vezes de -pi a pi e outras de 0 a 2 pi
6. Noises tão um bocado à toa, rever
7. Pose está a ser atualizada demasiadas vezes
8. Não saber a distribuição atual de pesos pelos clusters para fazer debug

# Soluções
1. Ir às partículas fora do mapa e inicializá-as aleatoriamente dentro
    1. Deixar isto e deixar a cena dos 10%
    2. Deixar isto e tirar a cena dos 10%
    3. Garantir que são sempre os 10%
2. Ruído uniforme de zero à percentagem de peso máximo
3. X Done (diminiuir espessura das rodas)
4. Done
5. Pôr tudo de -pi a pi
6. Noise dos weights é desnecessário, noise das medições deve ser proporcional à distância
7. Atualização da pose só deve ocorrer dentro do if dos deltas
8. Colorir as setas de acordo com os pesos/imprimir os pesos associados a cada cluster

# Cenas extra
1. LiDAR no robô
2. Deteção de cantos a partir da leitura e encaixar com o resto do filtro
3. Spawn aleatório
4. Serviço para reinicializar as partículas no mapa