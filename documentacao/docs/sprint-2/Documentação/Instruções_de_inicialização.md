---
sidebar_position: 2
custom_edit_url: null
title: "Instruções de Inicialização"
---

## Introdução

&emsp;&emsp; Este documento é crucial para qualquer usuário que deseje inicializar e operar eficientemente a Interface de Linha de Comando (CLI) e o sistema robótico associado. Contendo instruções detalhadas e meticulosas, ele serve como um guia fundamental para configurar corretamente ambos os componentes tecnológicos. Ao seguir este manual, espera-se que os sistemas operem com plena funcionalidade, otimizando sua performance operacional.

&emsp;&emsp; A aderência rigorosa aos procedimentos descritos é vital. Isso não apenas facilita uma configuração bem-sucedida, mas também previne possíveis falhas operacionais. A execução correta dessas etapas assegura que eventuais problemas sejam minimizados, permitindo que o usuário explore todo o potencial do sistema.

&emsp;&emsp; Portanto, é imprescindível seguir cuidadosamente cada passo apresentado neste documento. Fazer isso garantirá o melhor desempenho possível do equipamento, maximizando a eficiência e eficácia na utilização da CLI e do sistema robótico.
### Versões das Bibliotecas Necessárias

&emsp;&emsp; Para evitar problemas de compatibilidade e garantir a integridade do ambiente de execução, é imperativo utilizar as versões das bibliotecas conforme listado abaixo:

- **ROS2**: `humble`
- **Python**: `3.10.10`
- **Typer**: `0.9.0`
- **Inquirer**: `3.2.4`

## Preparação do Ambiente de Execução

### Configuração do Sistema Operacional

&emsp;&emsp; O software foi desenvolvido para ser executado especificamente no Ubuntu 22.04. Utilizar uma versão ou sistema operacional diferente pode resultar em falhas de execução.

### Verificação e Instalação das Dependências

&emsp;&emsp; Antes de prosseguir, é essencial verificar se as dependências principais estão corretamente instaladas em seu sistema. Utilize os comandos abaixo para confirmar as versões de Python e ROS2:

```bash
python3 --version

printenv ROS_DISTRO
```
#### Instalando Python

&emsp;&emsp; Se o Python não estiver presente em seu sistema, instale-o usando o comando:

```bash
sudo apt install python3
```

#### Instalando ROS2

&emsp;&emsp; Para instalar o ROS2, caso ainda não esteja configurado, siga as instruções detalhadas disponíveis neste [link](https://docs.ros.org/en/iron/Installation.html).

### Estabelecendo a Conexão de Rede

&emsp;&emsp; É crucial que seu computador esteja conectado à mesma rede Wi-Fi que o robô para permitir a comunicação.

### Inicialização da CLI

&emsp;&emsp; A inicialização correta da Interface de Linha de Comando (CLI) é fundamental para o controle efetivo do robô e para a execução de comandos específicos do sistema. Para começar, siga estes passos detalhadamente para garantir que a CLI seja iniciada sem problemas:


#### Configuração para o Bash

1. **Clonar o Repositório do Projeto:**
&emsp;&emsp; Abra um terminal e digite o seguinte comando para clonar o repositório do projeto para sua máquina local:

```bash
git clone https://github.com/Inteli-College/2024-1B-T08-EC06-G04.git
```
2. **Acessar o Diretório do Projeto:**
&emsp;&emsp; Navegue até o diretório do projeto utilizando o comando:

```bash
cd 2024-1B-T08-EC06-G04
```

3. **Execute o script de inicialização**
&emsp;&emsp; Execute o seguinte código para o robô "escutar" os publishers feitos pela CLI:
```bash
./exec2_bash.sh
```

&emsp;&emsp; Em uma nova aba do terminal, continue com a execução do segundo script para abrir a interface CLI em sua máquina:

```bash
./exec_bash.sh
```

#### Configuração para o Zsh

&emsp;&emsp; Para usuários do Zsh, os comandos são ligeiramente diferentes:

1. **Clonar o Repositório do Projeto:**
&emsp;&emsp; Abra um terminal e digite o seguinte comando para clonar o repositório do projeto para sua máquina local:

```bash
git clone https://github.com/Inteli-College/2024-1B-T08-EC06-G04.git
```
2. **Acessar o Diretório do Projeto:**
&emsp;&emsp; Navegue até o diretório do projeto utilizando o comando:

```bash
cd 2024-1B-T08-EC06-G04
```

3. **Execute o script de inicialização**
&emsp;&emsp; Execute o seguinte código para o robô "escutar" os publishers feitos pela CLI:

```bash
./exec2_zsh.sh
```

&emsp;&emsp; Em uma nova aba do terminal, continue com a execução do segundo script para abrir a interface CLI em sua máquina:

```bash
./exec_zsh.sh
```

&emsp;&emsp; Uma vez concluídos esses procedimentos, a Interface de Linha de Comando (CLI) estará totalmente operacional e configurada para facilitar a interação eficaz com o robô
