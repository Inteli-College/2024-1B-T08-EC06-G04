---
sidebar_position: 1
custom_edit_url: null
title: "Metodologia"
---

# Introdução

Aqui será apresentada a metodologia do que foi feito durante a terceira Sprint de desenvolvimento da solução com o sistema do Turtlebot, nesta documentação serão abordados os temas relacionados ao sistema `LiDAR` e segurança, além do sistema de aquisição de imagens do robô em tempo real.

:::tip Lembre-se
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

:::info Nota
Como ainda na Sprint 3, o projeto não foi concluído, as tecnologias ainda podem ser mudadas posteriormente.
:::

### Tópicos Utilizados Durante a Sprint 3

Seguindo o mesmo conceito de tópicos citados durante a [metodologia na Sprint 2](../../sprint-2/Documentação/Metodologia.md#tópicos), apresentamos o principal tópico utilizado nesta Sprint para capacitar o robô com a tecnologia `LiDAR`.

#### Tópico Principal

**/scan**: O tópico `scan` desempenha um papel crucial no funcionamento do robô equipado com `LiDAR`. Através deste tópico, o `LiDAR` envia os dados para o sistema ROS (Robot Operating System). Esses dados são indispensáveis para o controle da movimentação do robô, permitindo que ele pare automaticamente ao detectar obstáculos.

:::info Nota
📝 Para mais detalhes sobre a configuração do `LiDAR` e como o tópico `/scan` é implementado no ROS, consulte a [documentação técnica](../../sprint-2/Documentação/Metodologia.md#tópicos-e-nós).
:::

---

## Criação dos Novos Sistemas

A seguir, discutiremos como os novos sistemas de [`LiDAR`](#🚀-lidar) e [processamento de imagens](#📸-opencv) foram integrados ao projeto, juntamente com as principais partes do código relacionadas a esses sistemas.

:::info Nota
Este é um resumo das modificações realizadas. Para acessar o código fonte completo, visite o [GitHub do projeto](https://github.com/Inteli-College/2024-1B-T08-EC06-G04).
:::

### Adição do LiDAR no Código de Movimentação do Robô

Para integrar o `LiDAR` ao código do robô, foram necessárias várias modificações importantes. Entre elas, destacam-se:

- **Escuta do Tópico `scan`**: O robô foi configurado para escutar o tópico `scan` a fim de receber dados de distância fornecidos pelo `LiDAR`.
- **Implementação de Ferramenta de Obstrução**: Foi desenvolvida uma ferramenta que impede a movimentação do robô caso a frente esteja obstruída, evitando que ele avance. De forma similar, se a parte traseira estiver obstruída, o robô não poderá se mover para trás.


### Bibliotecas referentes ao LiDAR utilizadas

Aqui estão as novas bibliotecas que foram integradas ao projeto nesta sprint relacionadas ao LiDAR:

#### LaserScan (sensor_msgs.msg)
A biblioteca `LaserScan` é utilizada para processar os dados adquiridos através da escuta do tópico `scan`. Esta biblioteca é essencial para interpretar as leituras do `LiDAR` e garantir que os dados de distância sejam corretamente compreendidos pelo sistema.

#### qos_profile_sensor_data (rclpy.qos)
A biblioteca `qos_profile_sensor_data` foi usada para adicionar Quality of Service (QOS) ao tópico do ROS2. Isso garante a qualidade na transmissão dos dados sensoriais, permitindo que o robô reaja rapidamente às mudanças no ambiente.

:::info Nota
Todas as bibliotecas apresentadas na [**Sprint 2**](../../sprint-2/Documentação/Metodologia.md#bibliotecas-usadas) continuam sendo utilizadas nesta sprint.
:::

Segue abaixo o import exato das bibliotecas em python:

```python
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
```

:::tip
Nenhuma destas bibliotecas é necessário utilizar o pip para baixar. Ambas **ja fazem parte** do ambiente de execução do ROS2
:::

### Código atualizado referente a movimentação do robô

Abaixo está o código atualizado de movimentação do robô com a integração do LiDAR. As novas funcionalidades introduzidas serão discutidas detalhadamente a seguir, juntamente com a lógica por trás de sua implementação.

:::info Nota
Se seu robô não é capaz de interagir com o LiDAR, é possível seguir utilizando o código desenvolvido na [Sprint 2](https://github.com/Inteli-College/2024-1B-T08-EC06-G04/tree/0.2.0) do robô, porém **limitado** nas funções apresentadas naquela Sprint.
:::

```python
Codigo LIDAR :D
```

### Adição do Streamlit e Processamento de Imagens

Para proporcionar uma melhor visualização do robô, além de um sistema de controle mais otimizado e com uma UX aprimorada, desenvolvemos uma interface de usuário mais intuitiva e direta. Esta nova interface inclui:

- **Visualização em Tempo Real**: Uma câmera acoplada ao robô permite gravar e transmitir em tempo real o que o robô está fazendo, oferecendo uma visão completa das operações.
- **Botões de Controle Intuitivos**: Foram adicionados botões de controle mais intuitivos e fáceis de usar, facilitando a interação e o controle do robô.

A interface foi desenvolvida utilizando o Streamlit, que oferece uma plataforma eficiente para criar aplicações web interativas com Python.

### Bibliotecas Referentes ao Streamlit

Abaixo estão listadas as bibliotecas utilizadas para o funcionamento do Streamlit e para o processamento de imagens em tempo real:

- **cv2**: Utilizada para o processamento de imagens em tempo real.
- **base64**: Utilizada para codificação e decodificação de imagens.
- **numpy (np)**: Utilizada para operações matemáticas e manipulação de arrays.
- **streamlit (st)**: Utilizada para criar a interface web interativa.
- **PIL (Image)**: Utilizada para manipulação de imagens.
- **io**: Utilizada para operações de entrada e saída.

#### Código de Importação

Segue abaixo o código de importação dessas bibliotecas em Python:

```python
import cv2
import base64
import numpy as np
import streamlit as st
from PIL import Image
import io
```

:::info Nota
Exceto pelas bibliotecas `base64` e `io`, todas as outras bibliotecas mencionadas precisam ser instaladas utilizando o gerenciador de pacotes do Python, `pip`. As instruções detalhadas para a instalação estão disponíveis no guia de execução.
:::

### Código referente a interface visual e processamento de imagens

Os códigos abaixo são referentes a tanto a interface visual quanto ao processamento das imagens adquiridas pela câmera do robô. Afim de deixar mais claro como o mesmo foi executado e como pode ser alterado afim de abranger todas as necessidades.

:::warning
Qualquer alteração no código pode causar falhas ou mal-funcionamento do robô, altere o mesmo em seu próprio risco.
:::

O código abaixo é utilizado dentro do robô, onde o mesmo serve para passar as informações da webcam para o ambiente do ROS2 que contem o código do Streamlit na máquina pessoal.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import base64

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Erro: Não foi possível acessar a webcam.")
            return
        self.cap.set(cv2.CAP_PROP_FPS, 30)  # Configura a câmera para 60 FPS, se suportado
        timer_period = 0.0167  # aproximadamente 0.0167 segundos (60 FPS)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            _, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            msg = String()
            msg.data = jpg_as_text
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing image as base64 string')
        else:
            self.get_logger().error('Could not read image from webcam')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Abaixo será explicado melhor o código, suas classes e métodos.