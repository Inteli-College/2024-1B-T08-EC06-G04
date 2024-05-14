---
sidebar_position: 1
custom_edit_url: null
title: "Metodologia"
---

# Introdução

Aqui será apresentada a metodologia do que foi feito durante a terceira Sprint de desenvolvimento da solução com o sistema do Turtlebot, nesta documentação serão abordados os temas relacionados ao sistema `LiDAR` e segurança, além do sistema de aquisição de imagens do robô em tempo real.

:::tip
O `LiDAR` no TurtleBot 3 é um sensor de varredura a laser que mede distâncias até objetos em seu entorno. Ele emite pulsos de laser e calcula o tempo que cada pulso leva para refletir de volta ao sensor, permitindo a criação de um mapa 2D preciso do ambiente.
:::

Segue abaixo as etapas executadas para a realização da terceira Sprint.

## Tecnologias

Durante está sprint, serão utilizadas as seguintes tecnologias;

### 🚀 LiDAR
O LiDAR (Light Detection and Ranging) é um sensor que mede distâncias até objetos em seu entorno. Ele funciona emitindo pulsos de laser e calculando o tempo que cada pulso leva para refletir de volta ao sensor, permitindo a criação de um mapa 2D preciso do ambiente.

**Principais Funções do LiDAR no TurtleBot 3:**

- **Navegação Autônoma:** Ajuda o TurtleBot 3 a navegar de forma autônoma, evitando obstáculos e planejando rotas seguras.
- **Detecção de Obstáculos:** Detecta obstáculos ao redor do robô, permitindo reações a mudanças no ambiente e evitando colisões.

:::note
O `LiDAR` pode ser utilizado de outras maneiras também. Todavia até então usaremos o mesmo em nossa solução apenas para as funções descritas acima. Se quiser informações adicionais do que o LiDAR é capaz de realizar, recomendo ler este [artigo](https://www.faro.com/en/Resource-Library/Article/What-is-Lidar#:~:text=Lidar%20technology%20is%20an%20ideal,models%20and%20map%20digital%20elevation.).
:::

### 📸 OpenCV
OpenCV (Open Source Computer Vision Library) é uma biblioteca de visão computacional de código aberto que fornece uma ampla gama de ferramentas para processamento de imagem e vídeo.

**Principais Funções do OpenCV:**

- **Processamento de Imagem:** Filtragem, transformação, segmentação e análise das imagens adquiridas pelo robô.
- **Detecção de Objetos:** Algoritmos para reconhecimento e rastreamento de objetos, no caso do projeto será utilizada a detecção de entupimento nos canos.
- **Machine Learning:** Integração com modelos de aprendizado de máquina para tarefas de visão computacional.

### 🎥 Câmera do Dobot Magician
A câmera do Dobot Magician é usada para captura de imagens e vídeos, permitindo a interação visual com o ambiente.

**Principais Funções da Câmera do Dobot Magician:**

- **Captura de Imagem:** Obtenção de imagens estáticas do ambiente para análise.
- **Vídeo em Tempo Real:** Transmissão de vídeo ao vivo para monitoramento e controle.
- **Integração com Visão Computacional:** Utilizada junto com bibliotecas como OpenCV para detecção e reconhecimento de objetos.

### 🌐 Streamlit
Streamlit é uma biblioteca de código aberto para criação de aplicativos web interativos em Python, facilitando a visualização de dados e a criação de interfaces de usuário.

**Principais Funções do Streamlit:**

- **Criação de Interfaces de Usuário:** Ferramentas simples para construir interfaces de usuário interativas.
- **Visualização de Dados:** Integração fácil com bibliotecas de visualização como Matplotlib, Plotly e Altair.
- **Desenvolvimento Rápido:** Facilita o desenvolvimento rápido de protótipos e aplicativos.
- **Interatividade:** Permite a criação de widgets interativos para manipulação de dados em tempo real.

:::info
Como ainda na Sprint 3, o projeto não foi concluído, as tecnologias ainda podem ser mudadas posteriormente.
:::

