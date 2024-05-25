---
sidebar_position: 2
custom_edit_url: null
title: "Instruções de Execução"
---

# Introdução

&emsp;&emsp;Este documento é crucial para qualquer usuário que deseje inicializar e operar eficientemente a interface gráfica avançada e o sistema robótico associado. Contendo instruções detalhadas e meticulosas, ele serve como um guia fundamental para configurar corretamente ambos os componentes tecnológicos. Ao seguir este manual, espera-se que os sistemas operem com plena funcionalidade, otimizando sua performance operacional.

&emsp;&emsp;A aderência rigorosa aos procedimentos descritos é vital. Isso não apenas facilita uma configuração bem-sucedida, mas também previne possíveis falhas operacionais. A execução correta dessas etapas assegura que eventuais problemas sejam minimizados, permitindo que o usuário explore todo o potencial do sistema.

&emsp;&emsp;Portanto, é imprescindível seguir cuidadosamente cada passo apresentado neste documento. Fazer isso garantirá o melhor desempenho possível do equipamento, maximizando a eficiência e eficácia na utilização da interface gráfica e do sistema robótico.

## 1.0.0 Versões das bibliotecas necessárias:

:::danger
Caso julgue necessário, pode utilizar uma biblioteca em versão mais recente ou mais antiga. Todavia faça o mesmo em seu próprio **risco** pois pode acarretar em problemas na execução do código.
:::

- **pygame**: 2.3.0
- **cv2 (OpenCV)**: 4.5.5
- **numpy**: 1.24.2
- **PIL (Pillow)**: 9.2.0

## 2.0.0 Setup do ambiente

:::info
Para poder executar o código, é necessário ter uma versão de **python e ROS** em sua máquina local. Caso contrário não será possível a execução.
:::

### 2.1.0 Configuração do Sistema Operacional

&emsp;&emsp; O software foi desenvolvido para ser executado especificamente no Ubuntu 22.04. Utilizar uma versão ou sistema operacional diferente pode resultar em falhas de execução.

:::tip
Para checar sua versão atual do Ubuntu, digite no terminal o seguinte:
```bash
lsb_release -a
```

E veja se seguido na parte _Description_ está Ubuntu 22.04.04 LTS, se não estiver, será necessário reinstalar o sistema operacional em sua correta versão. Se necessário, segue o link com a versão correta do [Ubuntu](https://releases.ubuntu.com/jammy/)
:::

### 2.2.0 Instalação do python e ROS

Antes de prosseguir, é essencial verificar se tanto o python quanto ROS2 estão instalados em seu sistema. Utilize os comandos abaixo para confirmar as versões de Python e ROS2:

```bash
python3 --version

printenv ROS_DISTRO
```

Se os mesmos estiverem instalados já, pode prosseguir para a próxima etapa. Caso contrário siga o tutorial abaixo para instala-los.

#### 2.2.1 Instalando Python

Instale o mesmo em sua máquina Ubuntu 22.04 utilizando o seguinte código no terminal

```bash
sudo apt install python3
```

:::warning
O comando `sudo` sempre irá pedir a senha de sua máquina, para instalar aplicativos no Ubuntu, é necessário permissão de `root` ou seja, permissão total no sistema. Logo deve-se adicionar o sudo antes do comando.
:::

Após isto digite no terminal o seguinte para checar se o python está instalado

```bash
python3 --version
```

Se retornar python não encontrado, siga os passos de instalação novamente e reinicie seu sistema. Se retornar a versão de python pode prosseguir.

#### 2.2.2 Instalando o ROS2

&emsp;&emsp; Para a comunicação entre a interface de linha de comando (CLI) e o robô, será empregado o ROS2, uma escolha tecnológica estratégica que permite a transmissão eficiente de informações através de uma rede robusta. Este sistema é projetado para facilitar a interação entre dispositivos computacionais e máquinas autônomas, proporcionando uma plataforma confiável para o envio de comandos.

Dada a extensão da instalação e configuração do ROS2, é mais fácil seguir a documentação principal do mesmo afim de evitar erros.

:::tip
Para instalar o ROS2, caso ainda não esteja configurado, siga as instruções detalhadas disponíveis neste [link](https://docs.ros.org/en/foxy/Installation.html).
:::

## 3.0.0 Conexão rede - Mesma rede que robô

Afim de adquirir a conexão entre o robô e a máquina local para passar informações, é essencial que tanto o robô quanto a máquina que rodará o código da interface estejam na mesma rede Wi-Fi, podendo ser tanto uma rede local, quanto um hotspot.

:::warning
Certifique-se de que a rede não possua `travas` ou qualquer coisa do tipo que trave a comunicação entre o robô e a maquina local, lembrando que o protocolo de comunicação utilizado é o **ROS**
:::

Afim de conectar o robô em alguma rede, é necessário conectar um monitor teclado e mouse a Raspberry Pi do robô, e após isto conectar o mesmo a rede desejada, através do simbolo de rede localizado no canto superior direito.

:::tip
Para checar o ip de seu robô na rede deve-se digitar **ifconfig** no terminal do robô, podendo assim adquirir o IP do mesmo, futuramente será necessário.
:::

## 4.0.0 Inicialização do novo bringup no robô

A inicialização correta do bringup no robô é crucial para garantir que todos os sistemas estejam funcionando de maneira harmoniosa. No contexto do ROS2 e TurtleBot, o bringup refere-se ao processo de iniciar e configurar todos os nós, tópicos e serviços necessários para operar o robô. Seguir as etapas abaixo ajudará a garantir uma inicialização bem-sucedida:

:::info
Neste caso, foi desenolvido um novo bringup, afim de que ao pressionar o botão de emergência ele **encerra** o processo do bringup dentro do robô, fazendo com que seja impossível do mesmo continuar se movendo
:::

### 4.1.0 Fontar o workspace

Primeiramente, deve-se fontar o workspace do ROS2 em que o bringup está presente, para fazer o mesmo siga o tutorial abaixo a risca.

:::info
Fontar o ambiente no ROS significa executar o script `setup.bash` para configurar as variáveis de ambiente necessárias, permitindo que os comandos e pacotes do ROS funcionem corretamente.
:::

Primeiro entre no diretório do workspace
```bash
cd ~/main_ws
```

Após entrar no workspace deve-se fontar o mesmo.

```bash
source install/local_setup.bash
```

Se a parte de fontar der algum erro, execute o seguinte e logo em seguida, tente fontar novamente:

```bash
colcon build
```

### 4.2.0 Inicialização do arquivo de launch do ROS2

Após fontar o workspace, deve-se inicializar de fato o bringup, para fazer o seguinte digite os seguinte comandos.

Primeiro, entre no diretório launch
```bash
cd ~/main_ws/launch
```

Após entrar no diretório de launch, execute os arquivos de launch
```bash
ros2 launch launch.py
```

Após todos estes passos deve estar executando normalmente o bringup.

:::tip
Para checagem se o bringup de fato deu certo, cheque se o mesmo mostra no console que está enviando as imagens em formato **base64**.
::: 

## 5.0.0 Inicialização da interface

A inicialização correta da interface gráfica é fundamental para o controle efetivo do robô e para a execução de comandos específicos do sistema. Para começar, siga estes passos detalhadamente para garantir que a interface gráfica seja iniciada sem problemas

### 5.1.0 Clonagem do repositório do GitHub na máquina local

Primeiramente, deve-se clonar este repositório em sua máquina local, afim de realizar o mesmo, digite o seguinte comando em seu terminal.

:::tip
Lembrando, deve-se utilizar o **Ubuntu 22.04**
:::

```bash
git clone https://github.com/Inteli-College/2024-1B-T08-EC06-G04.git
```

### 5.2.0 Instalar as bibliotecas do python

Após clonar o reposítorio, deve-se instalar as bibliotecas necessárias do python. Utilize o comando abaixo para realizar o mesmo.

```bash
pip install pygame==2.3.0 opencv-python==4.5.5 numpy==1.24.2 pillow==9.2.0
```

### 5.3.0 Executar o código principal

Após realizar tais passos, deve-se executar o código principal da interface gráfica, siga os passos abaixo

```bash
cd 2024-1B-T08-EC06-G04/src/visor_v2
```

Execute o código

```bash
python3 main.py
```

Após isto, está pronto para utilização.

## 6.0.0 Operação da interface

### 6.1.0 Movimentação do Robô
- **Teclas de Direção**: Utilize `W`, `A`, `S`, `D` para movimentar o robô para frente, esquerda, trás e direita, respectivamente.
- **Parada Rápida**: Pressione a tecla `espaço` para parar o robô imediatamente em caso de travamento.
- **Parada de Emergência**: Pressione a tecla `B` para solicitar a parada de emergência e fechar o bringup.

### 6.1.0 Controle via Interface Gráfica
- **Botões de Direção**: Os quatro botões principais no centro da interface controlam as direções do robô (frente, esquerda, trás, direita).
- **Parada de Emergência**: O botão no canto superior esquerdo, sem ícones, solicita a parada de emergência.

### 6.2.0 Exibição de Câmera
- A interface gráfica exibe a visão da câmera em tela cheia, permitindo monitoramento em tempo real do ambiente ao redor do robô.

### 6.3.0 Latência
- A latência atual do sistema é exibida no canto superior direito da interface, proporcionando feedback sobre o desempenho da comunicação e resposta do robô.

:::info
Para fechar a interface basta clicar no X no canto superior direito para fechar aplicações normais de sua máquina. Se o mesmo por algum motivo não fechar, vá para o terminal em que o código do mesmo está sendo executado e digite a combinação de teclas **CTRL + C** repetidas vezes até finalizar completamente o processo
:::


## 7.0.0 Features do TurtleBot3

### 7.1.0 Câmera do Dobot Magician

O TurtleBot3 pode ser integrado com qualquer câmera do, adicionando capacidades avançadas de visão ao robô. Esta combinação permite ao TurtleBot3 realizar tarefas mais complexas de navegação e inspeção.

:::info
Em nosso caso, utilizamos a câmera do Dobot Magician que não oferece uma qualidade e resolução de imagem muito alta, todavia tem um baixo custo.
:::

#### 7.1.1 Câmera do Dobot Magician

A câmera do Dobot Magician oferece uma visão em tempo real do ambiente. Ela é usada para tarefas de reconhecimento de imagem, inspeção visual e outras aplicações que exigem análise visual. A câmera pode capturar imagens e transmitir o feed de vídeo para a interface gráfica, permitindo monitoramento e controle mais precisos.

:::important
**Transmissão em Tempo Real**: A capacidade de transmitir vídeo em tempo real para a interface gráfica permite uma supervisão eficiente e resposta rápida a mudanças no ambiente. Vale ressaltar que depende também de sua conexão com a rede, podendo aumentar ou diminuir a latência.
:::

:::info
Quanto maior a latência, maior o tempo de diferença entre a camerâ e o mundo real. Logo tente sempre buscar a menor latência possível.
:::


## 8.0.0 Cuidados

### 8.1.0 Cuidados na Operação do Robô

Para garantir a segurança e a longevidade do TurtleBot3 e de seus componentes, é importante seguir algumas precauções durante a operação e o desligamento do robô.

#### 8.1.1 Desligamento Adequado

1. **Desligar a Raspberry Pi**
   - Utilize o comando abaixo para desligar a Raspberry Pi de forma segura:
     ```bash
     sudo poweroff
     ```

2. **Desligar o Botão de Energia**
   - Após a Raspberry Pi ser desligada completamente, pressione o botão de energia no TurtleBot3 para desligá-lo.

3. **Remover a Bateria**
   - Somente depois de desligar a Raspberry Pi e o botão de energia, desconecte a bateria do TurtleBot3.

#### 8.1.2 Avisos de Bateria

- **Alarme de Bateria Fraca**
  - Se o robô começar a emitir um apito, isso indica que a bateria está ficando fraca e o TurtleBot3 deve ser desligado imediatamente para evitar danos.
  
:::caution
**Nível de Bateria Crítico**: Se a tensão da bateria cair abaixo de 11V, há risco de perda permanente da capacidade da bateria. Certifique-se de monitorar os níveis de tensão regularmente e carregar a bateria conforme necessário.
:::