# trabalho3_psr_robutler

  Para este trabalho da cadeira de PSR, foi desenvolvido um sistema robótico que funciona como mordomo. Este robô é capaz de realizar várias missões de apoio que habitualmente sao realizadas por humanos. Para a realização destas missões o robô pode percecionar objetos e movimentar-se por o cenário (casa).
  
  O robô desenvolvido é baseado num turtlebot waffle pi. No entanto, este foi modificado de modo a que seja possível ter uma maior quantidade de informação para cumprir as suas missões. As modificações efetuadas foram a elevação da câmara. O robô é capaz de identificar vários objetos (Cubo azul e esfera violeta, **) de acordo com os dados adquiridos pelo sensor.Este robô também pode tirar uma fotografia com a camara e procurar um objeto em uma determinada divisão. O robô também pode contar quantos objetos é que existem na casa. 
  
  O cenário foi mapeado de modo a ser utilizado offline como suporte á navegação autónoma. O robô pode deslocar-se por condução manual com teleop, por condução autónoma para um alvo expresso em coordenadas X,Y e por condução autónoma para um alvo expresso com informação semântica. Foi criada uma função para adicionar objetos à casa em posições não conhecidas pelo robô. Esta funçao permite usar argumentos para definir o que será criado e em que divisão do apartamento.


**É possivel adicionar uma pessoa na casa, mas esta não é reconhecida pelo robô.

  Para iniciar o robô é necessário inserir dois comandos no terminal
  
    roslaunch robutler_bringup_psr gazebo.launch

    roslaunch robutler_bringup_psr bringup.launch

  Estes dois comandos irão iniciar a simulação da casa e o rviz onde se irão ver as mensagens do robô e também controlá-lo.
  De seguida, para controlar o robô manualmente é necessário inserir o seguinte comando no terminal:

    roslaunch robutler_teleop robutler_teleop_key.launch

  Para inserir o menu interativo no robô de modo a poder realizar as missões, é necessário inserir o seguinte comando no terminal:

    roslaunch robutler_menu launch.launch
