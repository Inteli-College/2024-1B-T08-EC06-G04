---
sidebar_position: 1
custom_edit_url: null
title: "Metodologia"
---

## Introdução

&emsp;Neste documento, apresentamos a metodologia adotada para o desenvolvimento do sistema de robô móvel Turtlebot, com o objetivo de otimizar o processo de manutenção dos reboilers na Atvos. O projeto envolveu várias etapas, seguindo um fluxo claro e estruturado que será apresentado a seguir.

---

## Tecnologias

&emsp;A integração do ROS 2 com o TurtleBot3, além de ser fundamental para a autonomia e eficiência dos robôs em ambientes industriais, também se estende ao uso da câmera Dobot Magic e do YOLO para identificação de imagens. Essa ampliação tecnológica permite uma funcionalidade avançada de percepção visual, agregando inteligência ao sistema robótico.

&emsp;Com a câmera Dobot Magic acoplada ao TurtleBot3 e integrada ao ROS 2, podemos capturar imagens em tempo real do ambiente de trabalho. Essas imagens são processadas pelo YOLO (You Only Look Once), um sistema de detecção de objetos em tempo real baseado em deep learning. O YOLO é capaz de identificar e classificar objetos dentro das imagens, o que permite ao robô tomar decisões autônomas com base nas informações visuais coletadas.

&emsp;Ao combinar a navegação autônoma oferecida pelo ROS 2 com a percepção visual habilitada pela câmera Dobot Magic e pelo YOLO, o TurtleBot3 torna-se capaz de realizar tarefas complexas, como encontrar e manipular objetos específicos em seu ambiente. Isso amplia significativamente as capacidades do robô, possibilitando-o a executar uma variedade de atividades industriais de forma autônoma e eficiente.

&emsp;Dessa forma, a integração do ROS 2, TurtleBot3, câmera Dobot Magic e YOLO representa um avanço notável na aplicação de tecnologias de ponta para impulsionar a automação e a inteligência nas operações industriais. Essa solução integrada capacita os robôs a serem mais versáteis e adaptáveis, abrindo novas possibilidades para a automação inteligente em diversos contextos industriais, como a manutenção de reboilers na Atvos.

### Tópicos e nós

&emsp;No contexto do ROS 2 (Robot Operating System 2), os conceitos de tópicos e nós desempenham papéis fundamentais na comunicação e coordenação entre os componentes de um sistema robótico. Abaixo, exploramos o significado e a funcionalidade desses conceitos:
Tópicos

#### Tópicos

&emsp;Um tópico no ROS 2 é um canal de comunicação assíncrono utilizado para troca de mensagens entre nós (nodes) dentro do sistema. Ele permite que os nós publiquem mensagens em um tópico específico e que outros nós interessados se inscrevam nesse tópico para receber e processar essas mensagens.

- **Publicação:** Um nó que publica mensagens em um tópico é chamado de publisher. Ele envia periodicamente ou em resposta a eventos específicos mensagens contendo informações relevantes para outros nós.

- **Inscrição:** Um nó que recebe mensagens de um tópico é chamado de subscriber. Ele se inscreve em um tópico específico para receber e processar as mensagens publicadas por outros nós.

Os tópicos facilitam a comunicação entre os diferentes componentes de um sistema robótico, permitindo a troca de informações de forma eficiente e flexível.

##### Tópicos Utilizados:
**cmd_vel:**  Este é um tópico crucial no qual o sistema publica mensagens do tipo geometry_msgs/Twist. Ele permite controlar as velocidades lineares e angulares do TurtleBot, essenciais para o seu movimento autônomo e teleoperado.
**image_raw:** Outro tópico utilizado pode ser image_raw, onde as imagens capturadas pela câmera Dobot Magic são publicadas. Essas imagens são consumidas pelo nó responsável pela detecção de objetos (possivelmente integrado ao YOLO) para processamento e tomada de decisões.

#### Nós (Nodes)

&emsp;Um nó (node) no ROS 2 é um processo executável que realiza uma tarefa específica dentro do sistema robótico. Cada nó é independente e comunica-se com outros nós por meio de tópicos, serviços ou parâmetros.

- **Funcionalidade:** Cada nó pode desempenhar funções como controle de sensores, processamento de dados, planejamento de trajetórias, geração de mapas, entre outros.

- **Comunicação:** Os nós se comunicam trocando mensagens por meio dos tópicos. Um nó pode ser tanto um publisher (que envia mensagens) quanto um subscriber (que recebe mensagens) em um ou mais tópicos.

- **Flexibilidade:** A arquitetura baseada em nós permite que diferentes partes de um sistema robótico sejam desenvolvidas, testadas e modificadas de forma independente, facilitando a escalabilidade e manutenção do sistema como um todo.

##### Interação como Publisher:

- O nó robot_controller atua como um publisher no tópico cmd_vel. Ele envia mensagens do tipo geometry_msgs/Twist contendo velocidades linear e angular para controlar o movimento do TurtleBot.
- Se houver outros nós responsáveis por processar imagens (image_raw) ou dados de sensores, eles podem atuar como publishers em diferentes tópicos para compartilhar informações relevantes com outros componentes do sistema.

#### Interação como Subscriber:

- Os nós que precisam receber comandos de movimento do robô se inscrevem no tópico cmd_vel como subscribers. Por exemplo, o subsistema de controle de movimento do TurtleBot irá se inscrever nesse tópico para receber e interpretar as mensagens de velocidade enviadas pelo robot_controller.
- Outros nós, como os responsáveis pelo processamento de imagens para detecção de objetos, podem ser subscribers de tópicos que publicam dados de sensores ou imagens (image_raw). Isso permite que eles recebam os dados necessários para realizar suas tarefas de processamento.

---

## Metodologia

&emsp;Neste documento, descrevemos a metodologia utilizada no desenvolvimento do sistema de robô móvel TurtleBot para otimizar o processo de manutenção dos reboilers na Atvos. O projeto foi conduzido por meio de várias etapas estruturadas, as quais detalharemos a seguir.

### Integração CLI

&emsp; Afim de criar a CLI (Command Line Interface) que controla o TurtleBot3 utilizando o ROS (Robot Operating System), foi escolhida a linguagem python, devido a sua simplicidade e vasto escossistema de bibliotecas, facilitando o uso do mesmo.



#### Bibliotecas usadas

- **rlcpy:** 
RLCPY é a biblioteca padrão do python para interagir com o ROS2, a mesma é utilizada para inicializar o nó do ROS2, criar os publishers (publicadores) e gerenciar a comunicação com o ROS, permitindo que o script envie e receba mensagens dentro do workspace.

- **Typer:** 
A biblioteca Typer foi utilizada para a criação da CLI, pois ela além de definir comandos baseados em argumentos e entradas do usuário na hora de executar o script, ela auxilia na verificação de erros.

- **Inquirer:** 
Para melhorar o a Experiência do Usuário na CLI, foi utilizado o Inquirer para retornar um menu interativo estilo "select" afim de deixar o usuário selecionar entre as opções dentro de nossa CLI de modo mais fácil e rápido.



#### Comunicação com o TurtleBot

&emsp;**geometry_msg/Twist:** Utilizamos a mensagem Twist do pacote geometry_msgs que é um padrão em ROS para enviar comandos de movimento. No script da CLI, essa mensagem foi usada para especificar velocidades lineares e angulares que são publicadas no tópico cmd_vel, o canal padrão para controlar movimentos do robo.

```python
# Código relevante para comunicação com o TurtleBot

    [...]

    self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    [...]

    def move_robot(self):
        if self.connected:
            msg = Twist()
            msg.linear.x = self.linear_speed
            msg.angular.z = self.angular_speed
            self.publisher.publish(msg)
            print(f"Movendo: velocidade linear={round(self.linear_speed,2)} m/s, velocidade angular={round(self.angular_speed,2)} rad/s")
```
#### Estrutura do código

&emsp;O aplicativo inicia criando um nó ROS chamado robot_controller. Este nó é essencial para a interação com o sistema ROS, pois através dele são realizadas todas as operações de publicação e subscrição de mensagens. O principal meio de comunicação com o robô é através do publicador que envia mensagens para o tópico cmd_vel. Estas mensagens controlam diretamente a movimentação do robô ao especificar velocidades lineares e angulares.

```python
# Código relevante para a estrutura do código
class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.connected = False
        self.linear_speed = round(0.0,2)
        self.angular_speed = round(0.0,2)
        self.killed = False
```

#### Interface de usuário

&emsp;**Menu Interativo:** Utilizando a biblioteca Inquirer.py, o menu permite ao usuário escolher entre conectar ao robô, desconectar, entrar no modo de teleoperação, ou sair do programa. Esta parte é crucial para garantir que qualquer usuário, independentemente de sua familiaridade com ROS ou programação, possa operar o robô facilmente.


&emsp;**Modo de Teleoperação:** Quando o usuário seleciona a opção de teleoperar, ele pode controlar o robô em tempo real usando teclas específicas (como w, a, s, d, espaço). A entrada do teclado é tratada de forma não bloqueante usando bibliotecas específicas do sistema operacional (como tty e termios para sistemas UNIX-like e msvcrt para Windows), permitindo que os comandos sejam responsivos e imediatos.

```python
# Código relevante para a interface de usuário
def teleop_mode(robot_controller):
    settings = None  
    try:
        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)
        print("Entrando no modo de teleoperação. Use as seguintes teclas para controlar o robô:")
        print("  _______ ")
        print(" |   w   |")
        print(" | a s d |")
        print(" |_______|")
        print(" Use 'w', 's', 'a', 'd' para mover.")
        print(" Use 'espaço' para parar.")
        print(" Use 'q' para sair.")
        print(" Use 'b' para forçar a morte do processo e parada.")
        while True:
            key = get_key(settings)  
            if key == 'w':
                robot_controller.increase_linear_speed()
            elif key == 's':
                robot_controller.decrease_linear_speed()
            elif key == 'a':
                robot_controller.increase_angular_speed()
            elif key == 'd':
                robot_controller.decrease_angular_speed()
            elif key == ' ':
                robot_controller.stop_robot()
            elif key == 'b':
                robot_controller.kill_switch()
                break
            elif key == 'q':
                break    # Sair do modo teloperado
            time.sleep(0.1)  
    finally:
        if os.name != 'nt' and settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
```
#### Tratamento de Erros e Exceções

&emsp; Incluimos o monitoramento de exceções durante a execução do programa e a implementação de mecanismos de recuperação adequados para lidar com condições inesperadas.

```python
# Código relevante para tratamento de erros e exceções
except KeyboardInterrupt:
    robot_controller.disconnect()
    rclpy.shutdown()
    user_thread.join()
markdown
```

#### Segurança e Confiabilidade

&emsp;Incluimos mecanismos de prevenção de falhas e procedimentos de parada de emergência. Isso garante que o robô opere de forma segura em diferentes condições de operação e que possa ser controlado de maneira confiável pelos usuários.

```python
# Código relevante para segurança e confiabilidade
def kill_switch(self):
    self.killed = True
    self.linear_speed = round(0.0,2)
    self.angular_speed = round(0.0,2)
    self.move_robot()
```

