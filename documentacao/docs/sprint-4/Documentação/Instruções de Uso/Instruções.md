---
sidebar_position: 1
custom_edit_url: null
title: "Instruções de Execução"
---

# Introdução

&emsp;&emsp;Este documento é crucial para qualquer usuário que deseje inicializar e operar eficientemente o projeto desenvolvido até esta Sprint. Contendo instruções detalhadas e meticulosas, ele serve como um guia fundamental para configurar corretamente todos os componentes do sistema. Ao seguir este manual, espera-se que o projeto opere com plena funcionalidade, otimizando sua performance operacional.

&emsp;&emsp;A aderência rigorosa aos procedimentos descritos é vital. Isso não apenas facilita uma configuração bem-sucedida, mas também previne possíveis falhas operacionais. A execução correta dessas etapas assegura que eventuais problemas sejam minimizados, permitindo que o usuário explore todo o potencial do projeto.

&emsp;&emsp;Portanto, é imprescindível seguir cuidadosamente cada passo apresentado neste documento. Fazer isso garantirá o melhor desempenho possível do projeto, maximizando a eficiência e eficácia na sua utilização.

## 1.0.0 Versões das bibliotecas necessárias:

:::danger
Caso julgue necessário, pode utilizar uma biblioteca em versão mais recente ou mais antiga. Todavia faça o mesmo em seu próprio **risco** pois pode acarretar em problemas na execução do código.
:::

- **Yolo**: V8
- **FastAPI**: 0.111.0
- **Tiny DB**: 4.8.0
- **Uvicorn**: 0.30.1
- **Websockets**: 12.0
- **Ultralytics**: 8.2.28

## 2.0.0 Setup do ambiente

:::info
Para poder executar o código, **não é necessário** ter `Ubuntu 22.04` e `ROS2` em sua máquina, porém ter os mesmos irá **reduzir** a latência tanto da câmera quanto dos controles relacionados ao robô.
:::

&emsp;&emsp; É importantíssimo ter tanto o `python` quanto o `npm` instalados em sua máquina, mais abaixo será explicado corretamente como checar se ambos estão instalados. Também será explicado como checar a versão do Ubuntu e ROS, se os mesmos forem utilizados pela sua preferência.

### 2.1.0 Configuração do Sistema Operacional (Opcional)

&emsp;&emsp; O software foi desenvolvido para ser executado no Ubuntu 22.04 preferencialmente. Utilizar uma versão ou sistema operacional diferente pode resultar em uma execução mais lenta.

:::tip
Para checar sua versão atual do Ubuntu, digite no terminal o seguinte:
```bash
lsb_release -a
```

E veja se seguido na parte _Description_ está Ubuntu 22.04.04 LTS, se não estiver, será necessário reinstalar o sistema operacional em sua correta versão. Se necessário, segue o link com a versão correta do [Ubuntu](https://releases.ubuntu.com/jammy/)
:::

!!Lembrando, **não** é necessário ter o `Ubuntu` e nem o `ROS`!!

### 2.2.0 Instalação do ROS (Opcional)

Antes de prosseguir, verifique se o ROS2 está instalado em seu sistema. Utilize o comando abaixo para confirmar a instalação do mesmo.

```bash
printenv ROS_DISTRO
```

Se ele já estiver instalado, pode pular para a próxima etapa, caso ele não estiver instalado, siga o tutorial abaixo.

### 2.2.1 Instalando ROS (Opcional)

Afim de instalar o ROS2 em seu S.O, primeiro certifique-se de estar no Ubuntu 22.04, caso contrário não funcionará.

&emsp;&emsp; Para a comunicação entre a interface de linha de comando (CLI) e o robô, será empregado o ROS2, uma escolha tecnológica estratégica que permite a transmissão eficiente de informações através de uma rede robusta. Este sistema é projetado para facilitar a interação entre dispositivos computacionais e máquinas autônomas, proporcionando uma plataforma confiável para o envio de comandos.

Dada a extensão da instalação e configuração do ROS2, é mais fácil seguir a documentação principal do mesmo afim de evitar erros.

:::tip
Para instalar o ROS2, caso ainda não esteja configurado, siga as instruções detalhadas disponíveis neste [link](https://docs.ros.org/en/foxy/Installation.html).
:::

### 2.3.0 Instalação do Python

Antes de prosseguir, é essencial verificar se o python está instalado em seu sistema. Utilize os comandos abaixo para verificar a instalação do python

```bash
python3 --version
```

Se o mesmo retornar algum erro, será necessário seguir o tutorial abaixo para instalar o python, caso contrário, pule a próxima etapa.

### 2.3.1 Instalando Python

Execute o seguinte comando para instalar o Python no Ubuntu 22.04 ou MacOs

Ubuntu 22.04

```bash
sudo apt install python3
```

:::warning
O comando `sudo` sempre irá pedir a senha de sua máquina, para instalar aplicativos no Ubuntu, é necessário permissão de `root` ou seja, permissão total no sistema. Logo deve-se adicionar o sudo antes do comando.
:::

MacOs

```bash
brew install python
```

Windows

Acesse o seguinte [link](https://www.python.org/downloads/windows/) e siga os tutoriais nele para baixar o `.exe` referente ao python.

Após isto digite no terminal o seguinte para checar se o python está instalado

```bash
python3 --version
```

Se retornar python não encontrado, siga os passos de instalação novamente e reinicie seu sistema. Se retornar a versão de python pode prosseguir. Se seguir sem retornar, tente checar a documentação oficial do python em seu sistema operacional afim de procurar por erros.

## 3.0.0 Conexão rede - Mesma rede que robô

Afim de adquirir a conexão entre o robô e a máquina local para passar informações, é essencial que tanto o robô quanto a máquina que rodará o código da interface estejam na mesma rede Wi-Fi, podendo ser tanto uma rede local, quanto um hotspot.

:::warning
Certifique-se de que a rede não possua `travas` ou qualquer coisa do tipo que trave a comunicação entre o robô e a maquina local, lembrando que o protocolo de comunicação utilizado é o **ROS**
:::

Afim de conectar o robô em alguma rede, é necessário conectar um monitor teclado e mouse a Raspberry Pi do robô, e após isto conectar o mesmo a rede desejada, através do simbolo de rede localizado no canto superior direito.

:::tip
Para checar o ip de seu robô na rede deve-se digitar **ifconfig** no terminal do robô, podendo assim adquirir o IP do mesmo, futuramente será necessário.
:::

## 4.0.0 Inicialização do bringup no robô

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

## 5.0.0 Inicialização do Websocket

Afim de compartilhar as informações entre o robô e o react foi utilizado o Rosbridge, que cria um websocket entre o robô e a interface visual.

Há também dois métodos de inicializar no Websocket, se o usuário tem o ROS2 e Ubuntu, deverá inicializar o mesmo em sua máquina local, seguindo o tutorial de numeração **5.1.1**.

Caso contrário, siga o tutorial de numeração **5.1.0** onde o Websocket é aberto dentro do robô.

### 5.1.0 Inicialização Websocket no robô

Afim de iniciar o Websocket dentro do robô, deve-se seguir os passos abaixo. Além disso, se certifique-se de estar em um terminal ou `.bash` ou `.zsh`.

Primeiramente conecte-se via `SSH` com o robô, para isso execute o seguinte código

```bash
ssh grupo4@10.128.0.9
```

:::info
Se pedir senha digite `grupo412`
:::

Após isto, rode no terminal do mesmo o seguinte comando para abrir o Websocket

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

E aguarde até o mesmo abrir na porta padrão: `9090`, após isto pode seguir com as instruções.

### 5.1.1 Inicialização Websocket na máquina local

Se o Ubuntu 22.04 e o ROS2 estão instalados em sua máquina, apenas digite o seguinte comando em seu terminal

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Após isto, seu Websocket estará aberto na porta padrão.

## 5.0.0 Inicialização da interface

A inicialização correta da interface gráfica é fundamental para o controle efetivo do robô e para a execução de comandos específicos do sistema. Para começar, siga estes passos detalhadamente para garantir que a interface gráfica seja iniciada sem problemas

### 5.1.0 Clonagem do repositório do GitHub na máquina local

Primeiramente, deve-se clonar este repositório em sua máquina local, afim de realizar o mesmo, digite o seguinte comando em seu terminal.

```bash
git clone https://github.com/Inteli-College/2024-1B-T08-EC06-G04.git
```


### 5.2.0 Inicialização do backend

Afim de inicializar o backend, é necessário primeiramente ter todas as bibliotecas instaladas em sua máquina. Para fazer o mesmo, siga os passos abaixo. 

:::danger
Se as bibliotecas não estiverem instaladas, o backend **não** poderá ser aberto e resultará em algum erro
:::

Primeiramente instale o uvicorn com o seguinte comando

```bash
pip install uvicorn
```

Após isto, instale o resto das bibliotecas com o seguinte comando

```bash
pip install fastapi ultralytics opencv-python tinydb
```

Após isto, navegue até o diretório `SRC` do backend, com o seguinte comando

:::info
Certifique-se de já estar no diretório deste projeto
:::

```bash
cd src/backend/src
```

Após entrar no diretório principal do backend, execute o seguinte código para iniciar a FastAPI

```bash
uvicorn app:app --reload
```

Após isto, seu backend estará ligado e funcional. Vale lembrar que o mesmo irá armazenar todas as `fotos` tiradas pelo robô.

### 5.3.0 Inicialização da aplicação

Afim de inicializar o website com a interface do robô, será necessário inicializar o React criado. Para fazer tal feito, deve-se digitar os seguintes códigos em seu terminal

Primeiramente vá ao diretório correto para inicialização

```bash
cd src/react/ProjetoMod6
```

Após ir até o diretório correto, execute o seguinte comando para instalar todas as depêndencias.

```bash
npm i
```

Após isto, inicie a interface web com o seguinte comando

```bash
npm run dev
```

Após todos estes passos, a interface está pronta para ser utilizada, caso algum problema tenha ocorrido, repasse sobre todas as etapas novamente.

:::warning
As instruções de operação seguem abaixo, para operar o robô do modo correto, **siga** os mesmos.
:::

## 6.0.0 Operações de uso