---
sidebar_position: 1
custom_edit_url: null
title: "Mudança da interface para React"
---

# Introdução

Durante a última Sprint, nossa equipe enfrentou um desafio significativo relacionado à escolha da interface visual para nosso projeto. Inicialmente, optamos por utilizar o Streamlit devido à sua facilidade de uso e rápida implementação. No entanto, logo percebemos que a falta de opções de customização desse framework limitava nossa capacidade de criar uma interface mais profissional e adaptada às necessidades específicas do projeto.

Diante dessa limitação, decidimos explorar alternativas que pudessem oferecer maior flexibilidade e um nível mais elevado de personalização. Após uma análise criteriosa das opções disponíveis, escolhemos o React como a nova base para o desenvolvimento de nossa interface visual. O React se destacou por sua ampla gama de possibilidades de customização, permitindo-nos criar uma página web robusta e esteticamente agradável.

Essa mudança foi motivada pela necessidade de proporcionar uma experiência de usuário superior, com uma interface visualmente mais atraente e funcional. O React nos oferece não apenas a capacidade de personalizar cada aspecto da interface, mas também uma estrutura modular que facilita a manutenção e a escalabilidade do projeto.

Em resumo, a transição do Streamlit para o React reflete nosso compromisso em buscar soluções mais eficazes e profissionais, garantindo que o produto final atenda às expectativas e necessidades dos usuários. Essa decisão estratégica aprimorará significativamente a qualidade e a usabilidade da nossa interface, resultando em um projeto mais refinado e alinhado com os padrões atuais de desenvolvimento web.

:::warning
Utilizar a interface via Pygame, ainda é possível, todavia a mesma tem uma carência em *UX*, afim de ter todo o potencial do projeto, é fortemente recomendado utilizar via React.
:::

## **Novas Tecnologias**

Durante a última Sprint, implementamos várias tecnologias avançadas para aprimorar a funcionalidade e a eficiência do nosso projeto na parte da interface visual. Entre as principais tecnologias utilizadas, destacam-se:

### 1. React
   - Desenvolvemos a nova interface do usuário utilizando React.
   - A escolha do React foi baseada na sua capacidade de criar interfaces interativas e altamente personalizáveis.

### 2. Tailwind CSS
   - Integrado com o React, o Tailwind CSS foi utilizado para estilizar a interface.
   - Esta biblioteca de CSS utilitária permitiu uma customização rápida e eficiente, resultando em uma interface moderna e responsiva.

### 3. ROSbridge
   - Implementamos o ROSbridge para a transmissão de imagens capturadas pelo robô.
   - Esta tecnologia permite a comunicação entre o sistema do robô e a interface web, facilitando a visualização em tempo real.

Essas tecnologias foram fundamentais para melhorar a transmissão de dados em tempo real e a criação de uma interface de usuário atraente e funcional.


:::info
As tecnologias referentes a outros serviços estarão em suas respectivas partes desta documentação
:::

## **Criação do React**

Para o desenvolvimento da interface do usuário, decidimos utilizar o React devido às suas capacidades robustas e flexíveis para construção de interfaces modernas e interativas. O processo de criação envolveu várias etapas importantes:

### 1. Configuração do Ambiente de Desenvolvimento
   - Inicialmente, configuramos o ambiente de desenvolvimento utilizando o Create React App, que fornece uma estrutura básica e as ferramentas necessárias para começar rapidamente.
   - Instalamos todas as dependências essenciais, incluindo bibliotecas adicionais que facilitariam o desenvolvimento e a integração com outras tecnologias.

### 2. Estruturação do Projeto
   - Organizamos o projeto em componentes modulares, garantindo que cada parte da interface fosse independente e reutilizável.
   - Essa modularidade facilita a manutenção do código e a implementação de novas funcionalidades no futuro.

### 3. Integração com ROSbridge
   - Implementamos a comunicação com o ROSbridge para receber e exibir as imagens capturadas pelo robô em tempo real.
   - Desenvolvemos componentes específicos para tratar os dados recebidos e atualizá-los dinamicamente na interface.

### 4. Estilização com Tailwind CSS
   - Utilizamos o Tailwind CSS para estilizar a interface, aproveitando suas classes utilitárias para aplicar estilos de forma rápida e eficiente.
   - Tailwind CSS permitiu-nos criar uma interface moderna e responsiva, atendendo às expectativas de usabilidade e estética.

### 5. Desenvolvimento de Funcionalidades Interativas
   - Implementamos diversas funcionalidades interativas, como a exibição em tempo real das imagens, controles para a navegação do robô e feedback visual sobre o status das operações.
   - Utilizamos hooks e estados do React para gerenciar a interação do usuário e a atualização dos dados na interface.

A criação da interface com React, complementada pelo uso do Tailwind CSS, resultou em um produto final robusto, moderno e altamente funcional, capaz de atender às necessidades específicas do projeto e proporcionar uma experiência de usuário superior.

:::info
O projeto foi criado levando em consideração a **escalablidade** do código, ou seja, o mesmo é facilmente escalável e reutilizável.
:::

## **Bibliotecas referentes ao React utilizadas**

Para o desenvolvimento da interface utilizando React, empregamos algumas bibliotecas e funcionalidades específicas que foram essenciais para atender aos requisitos do projeto. As principais bibliotecas e hooks utilizados foram:

### 1. roslib
   - Utilizamos a biblioteca `roslib` para integrar o React com o ROSbridge. Esta biblioteca permitiu a comunicação entre o frontend e o sistema do robô, facilitando a transmissão e a recepção de dados em tempo real.
   - `roslib` foi fundamental para estabelecer a conexão WebSocket com o ROSbridge, permitindo-nos enviar comandos ao robô e receber imagens e outros dados necessários para a interface.

### 2. React Hooks
   - **useState**: Utilizamos o hook `useState` para gerenciar o estado local dos componentes React. Este hook foi essencial para armazenar e atualizar dados dinâmicos, como as imagens recebidas do robô e o status das operações.
   - **useEffect**: O hook `useEffect` foi empregado para lidar com efeitos colaterais no React. Este hook nos permitiu executar código assíncrono e reagir a mudanças de estado ou props, como a conexão inicial com o ROSbridge e a atualização das imagens em tempo real.

Essas bibliotecas e hooks foram cruciais para o desenvolvimento de uma interface dinâmica e responsiva, permitindo-nos gerenciar eficientemente o estado da aplicação e integrar a comunicação com o sistema do robô. A combinação de `roslib` com os hooks `useState` e `useEffect` proporcionou uma base sólida para a criação de uma interface interativa e funcional, atendendo às necessidades do projeto de forma eficaz.


## **Estrutura e Explicação do Código React**

A seguir, detalharemos a estrutura do código React utilizado no projeto, separando-o em `pages`, `components` e `App.jsx`. Cada parte será explicada para facilitar a compreensão da sua funcionalidade e organização.

### 1. Páginas (pages)

Nesta seção, armazenamos as páginas principais da aplicação, que são componentes de alto nível representando diferentes views da nossa interface.

Estás são responsáveis por juntar todos os componenentes e preparar tudo em um arquivo afim de mostrar na tela de fato.

#### MainPage.jsx

- **Descrição**: *MainPage.jsx* é a página principal da aplicação, responsável por gerenciar a conexão com o ROSbridge e renderizar os componentes principais da interface, incluindo a câmera, o cabeçalho e os controles.
- **Função**: 
  - Estabelecer e gerenciar a conexão com o servidor ROSbridge.
  - Renderizar componentes principais (`Camera`, `Header`, `Controls`).
  - Manter o estado de conexão e passá-lo para os componentes filhos.
- **Hooks Utilizados**: `useState`, `useEffect`
- **Bibliotecas Utilizadas**: `ROSLIB`

##### Código Explicado:

```jsx
import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import Header from '../components/Header';
import Camera from '../components/Camera';
import Controls from '../components/Controls';

const MainPage = () => {
  // Estado para armazenar a instância do ROS e o status da conexão
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    // Cria uma nova instância do ROSLIB conectado ao servidor ROSbridge
    const rosInstance = new ROSLIB.Ros({ url: 'ws://10.128.0.9:9090' });

    // Define callbacks para eventos de conexão, erro e fechamento
    rosInstance.on('connection', () => {
      console.log('Connected to rosbridge websocket server.');
      setConnected(true);
    });

    rosInstance.on('error', (error) => {
      console.error('Error connecting to websocket server:', error);
      setConnected(false);
    });

    rosInstance.on('close', () => {
      console.log('Connection to websocket server closed.');
      setConnected(false);
    });

    // Armazena a instância do ROS no estado
    setRos(rosInstance);

    // Fecha a conexão ROS ao desmontar o componente
    return () => {
      rosInstance.close();
    };
  }, []);

  return (
    <div className="App flex flex-col justify-center min-h-screen bg-white w-full" tabIndex="0">
      <div className="grid grid-cols-8 gap-4 w-full p-4">
        <div className="col-start-2 col-span-1">
          <Camera ros={ros} />
        </div>
        <div className="col-start-4 col-span-1 mt-[-2rem]">
          <Header connected={connected} />
        </div>
        <div className="col-start-5 col-span-1 mt-20">
          <Controls ros={ros} />
        </div>
      </div>
    </div>
  );
};

export default MainPage;
```

Este código é responsável tanto por abrir o websocket para transmissão e recebimento de dados, quanto por juntar todos os componenentes, do Header, Controls e Camera dentro do `HTML` da página.

### 2. Componenentes (componenents)

Aqui, armazenamos os componentes reutilizáveis e modulares da nossa aplicação. Esses componentes são partes menores da interface que podem ser combinadas para criar páginas completas.

:::warning
Os mesmos são facilmente alterados, além de que podem ser facilmente adicionado novos. **Facilitando a escalabilidade ou alteração** de algo no projeto futuramente.
:::

#### Camera.jsx

- **Descrição**: *Camera.jsx* é um componente que se conecta ao ROSbridge e exibe a imagem capturada pelo robô.
- **Função**:
  - Subscrever ao tópico de vídeo do ROSbridge.
  - Processar as mensagens recebidas para extrair e exibir imagens.
  - Calcular e exibir a latência das imagens recebidas.
- **Hooks Utilizados**: `useState`, `useEffect`
- **Bibliotecas Utilizadas**: `ROSLIB`

##### Código Explicado:

```jsx
import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

const Camera = ({ ros }) => {
  const [frames, setFrames] = useState([]);
  const [timestamp, setTimestamp] = useState(null);
  const [latency, setLatency] = useState(null);

  useEffect(() => {
    if (!ros) return;

    // Define o tópico de vídeo no ROS
    const videoTopic = new ROSLIB.Topic({
      ros,
      name: '/chatter',
      messageType: 'std_msgs/String'
    });

    // Função de callback para processar mensagens de vídeo
    const handleMessage = (message) => {
      try {
        const [timestamp, base64Image] = message.data.split('|');
        const imgSrc = `data:image/jpeg;base64,${base64Image}`;
        const messageTimestamp = parseFloat(timestamp) * 1000; // Converte para milissegundos
        const currentTimestamp = Date.now();
        const latency = currentTimestamp - messageTimestamp;

        // Atualiza o estado dos frames
        setFrames((prevFrames) => {
          const newFrames = [...prevFrames, imgSrc];
          if (newFrames.length > 10) {
            newFrames.shift(); // Remove o frame mais antigo
          }
          return newFrames;
        });
        setTimestamp(new Date(messageTimestamp).toLocaleString());
        setLatency(latency);
      } catch (error) {
        console.error('Error processing image message:', error);
      }
    };

    // Subscreve ao tópico de vídeo
    videoTopic.subscribe(handleMessage);

    // Cancela a subscrição ao desmontar o componente
    return () => {
      videoTopic.unsubscribe(handleMessage);
    };
  }, [ros]);

  return (
    <div className="camera w-80 h-96 bg-gray-300 border-4 border-gray-500 rounded-lg flex flex-col items-center justify-center overflow-hidden">
      <img id="videoStream" src={frames[frames.length - 1]} alt="Video Stream" className="max-w-full max-h-full object-cover" />
      {timestamp && <div className="timestamp mt-2 text-sm text-gray-700">{`Timestamp: ${timestamp}`}</div>}
      {latency !== null && <div className="latency mt-1 text-sm text-gray-700">{`Latency: ${latency.toFixed(2)} ms`}</div>}
    </div>
  );
};

export default Camera;
```

O código acima é essencial pois é o mesmo que irá adquirir os dados enviados pelo robô, decodificá-los via `Base64` e transforma-los em uma imagem, alterando a mesma toda hora para a mesma se transformar em um vídeo.

#### Controls.jsx

- **Descrição**: *Controls.jsx* é um componente que fornece controles de movimento para o robô, permitindo a interação através do ROSbridge.
- **Função**:
  - Gerenciar e enviar comandos de movimento para o robô.
  - Monitorar as teclas pressionadas pelo usuário para determinar a direção do movimento.
  - Utilizar dados do LiDAR para evitar colisões, determinando se a área em frente ou atrás do robô está livre.
- **Hooks Utilizados**: `useState`, `useEffect`
- **Bibliotecas Utilizadas**: `ROSLIB`, `react-icons`

##### Código Explicado:

```jsx
import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { FaArrowUp, FaArrowDown, FaArrowLeft, FaArrowRight } from 'react-icons/fa';

const Controls = ({ ros }) => {
  const [cmdVel, setCmdVel] = useState(null);
  const [frontClear, setFrontClear] = useState(true);
  const [backClear, setBackClear] = useState(true);
  const [pressedKeys, setPressedKeys] = useState({});

  useEffect(() => {
    if (!ros) return;

    // Configura o tópico cmd_vel para enviar comandos de movimento
    const cmdVelTopic = new ROSLIB.Topic({
      ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });

    // Configura o tópico scan para receber dados do LiDAR
    const lidarTopic = new ROSLIB.Topic({
      ros,
      name: '/scan',
      messageType: 'sensor_msgs/LaserScan'
    });

    lidarTopic.subscribe(message => {
      lidarCallback(message.ranges);
    });

    setCmdVel(cmdVelTopic);

    // Funções para gerenciar teclas pressionadas
    const handleKeyDown = (e) => {
      setPressedKeys(prevKeys => ({ ...prevKeys, [e.key]: true }));
    };

    const handleKeyUp = (e) => {
      setPressedKeys(prevKeys => ({ ...prevKeys, [e.key]: false }));
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      lidarTopic.unsubscribe();
    };
  }, [ros]);

  // Atualiza o movimento do robô com base nas teclas pressionadas e na detecção de obstáculos
  useEffect(() => {
    const interval = setInterval(() => {
      updateMovement();
    }, 10);

    return () => {
      clearInterval(interval);
    };
  }, [frontClear, backClear, pressedKeys]);

  // Callback para processar dados do LiDAR
  const lidarCallback = (ranges) => {
    const numRanges = ranges.length;
    const sectorSize = Math.floor(numRanges / 12);
    const safetyDistance = 0.35;

    const frontLeftIndices = Array.from({ length: sectorSize }, (_, i) => numRanges - sectorSize + i);
    const frontRightIndices = Array.from({ length: sectorSize }, (_, i) => i);
    const backIndices = Array.from({ length: sectorSize * 2 }, (_, i) => 5 * sectorSize + i);

    const frontRanges = frontLeftIndices.concat(frontRightIndices).map(index => ranges[index]).filter(range => range > 0.01 && range < 100.0);
    const backRanges = backIndices.map(index => ranges[index]).filter(range => range > 0.01 && range < 100.0);

    const frontIsClear = !frontRanges.some(range => range < safetyDistance);
    const backIsClear = !backRanges.some(range => range < safetyDistance);

    setFrontClear(frontIsClear);
    setBackClear(backIsClear);
  };

  // Função para atualizar o movimento do robô
  const updateMovement = () => {
    let linear = 0.0;
    let angular = 0.0;

    if (frontClear && pressedKeys['w']) {
      linear = 0.2;
    } else if (backClear && pressedKeys['s']) {
      linear = -0.2;
    }

    if (pressedKeys['a']) {
      angular = 0.5;
    } else if (pressedKeys['d']) {
      angular = -0.5;
    }

    if (!frontClear && linear > 0) {
      linear = 0.0;
    }
    if (!backClear && linear < 0) {
      linear = 0.0;
    }

    moveRobot(linear, angular);
  };

  // Função para enviar comandos de movimento para o robô
  const moveRobot = (linear, angular) => {
    if (!cmdVel) return;

    const twist = new ROSLIB.Message({
      linear: { x: linear, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: angular }
    });

    cmdVel.publish(twist);
  };

  return (
    <div className="controls flex flex-col items-center space-y-2 ml-10">
      <button onClick={() => setPressedKeys({ 'w': true })} className="bg-blue-500 hover:bg-blue-700 text-white p-4 rounded-full">
        <FaArrowUp />
      </button>
      <div className="flex space-x-2">
        <button onClick={() => setPressedKeys({ 'a': true })} className="bg-blue-500 hover:bg-blue-700 text-white p-4 rounded-full">
          <FaArrowLeft />
        </button>
        <button onClick={() => setPressedKeys({ 'stop': true })} className="bg-blue-500 hover:bg-blue-700 text-white p-4 rounded-full">
          Parar
        </button>
        <button onClick={() => setPressedKeys({ 'd': true })} className="bg-blue-500 hover:bg-blue-700 text-white p-4 rounded-full">
          <FaArrowRight />
        </button>
      </div>
      <button onClick={() => setPressedKeys({ 's': true })} className="bg-blue-500 hover:bg-blue-700 text-white p-4 rounded-full">
        <FaArrowDown />
      </button>
    </div>
  );
};

export default Controls;
```

O código referente ao componente dos contoles do robô é essencial, tanto para possuir a lógica de adicionar os botões de movimentação a tela, o mesmo também possui uma funçõa de movimentar o robô através das teclas do teclado, facilitando a utilização.

Ele também envia as informações via tópico ROS para movimentar de fato o robô.

#### Header.jsx

- **Descrição**: *Header.jsx* é um componente que exibe o cabeçalho da aplicação e indica o status de conexão com o robô.
- **Função**:
  - Exibir o título da aplicação.
  - Mostrar uma mensagem de alerta se o robô não estiver conectado.
- **Hooks Utilizados**: Nenhum
- **Bibliotecas Utilizadas**: Nenhuma

##### Código Explicado:

```jsx
import React from 'react';

const Header = ({ connected }) => {
  return (
    <header className="App-header text-center w-full mb-8 ">
      <h1 className="text-4xl font-bold mb-4">Teleoperação do Robô</h1>
      {!connected && <div className="popup bg-red-500 text-white p-2 rounded shadow-lg fixed top-5 right-5">Robô não conectado</div>}
    </header>
  );
};

export default Header;
```

Este código é feito apenas para notificar o usuário se o robô está de fato conectado ou não através de um Pop-Up no canto direito superior da tela.

#### App.jsx

- **Descrição**: *App.jsx* configura a aplicação para usar o React Router, permitindo a navegação entre diferentes páginas.
- **Função**: 
  - Define a estrutura de roteamento da aplicação.
  - Renderiza a página principal (`MainPage`).
- **Bibliotecas Utilizadas**: `react-router-dom`

##### Código Explicado:

```jsx
import React from 'react';
import { BrowserRouter, Routes, Route } from 'react-router-dom';
import MainPage from './pages/MainPage';

const App = () => {
  return (
    <BrowserRouter>
      <Routes>
        <Route path="/" element={<MainPage />} />
      </Routes>
    </BrowserRouter>
  );
};

export default App;
```
Este código do App.jsx é responsável por adicionar as rotas as respectivas páginas.

:::info
Como exemplo, no caso acima, a página **MainPage** está na rota `http://localhost:3000/`
:::

### 3. Conclusão

A estrutura do código React é organizada de maneira modular, facilitando a manutenção e a expansão futura da aplicação. As `Páginas` contêm os componentes de alto nível que representam diferentes views, os `Componentes` são partes reutilizáveis da interface, e `App.jsx` organiza a estrutura e a navegação da aplicação. Essa organização permite uma implementação clara e escalável, atendendo às necessidades do projeto de forma eficiente.

## **Conclusão Final**

A transição de Streamlit para React trouxe uma série de melhorias significativas para o nosso projeto, proporcionando uma interface mais robusta, moderna e personalizável. Com a utilização de tecnologias como React, Tailwind CSS e ROSbridge, conseguimos desenvolver uma interface de usuário interativa e funcional que atende plenamente às necessidades do projeto.

### Benefícios da Mudança:

1. **Flexibilidade e Personalização**: 
   - O React permitiu-nos criar uma interface altamente customizável, com componentes modulares e reutilizáveis, facilitando a manutenção e a escalabilidade futura do projeto.

2. **Estilização Moderna**: 
   - O Tailwind CSS possibilitou uma estilização rápida e eficiente, resultando em uma interface visualmente atraente e responsiva, melhorando a experiência do usuário.

3. **Integração em Tempo Real**: 
   - Com o ROSbridge, conseguimos integrar a transmissão de dados do robô em tempo real, essencial para a visualização e controle do robô através da interface web.

4. **Organização Modular**: 
   - A estrutura modular do código, dividida em páginas e componentes, facilita a compreensão, manutenção e expansão do projeto. Cada parte do sistema foi projetada para ser independente e reutilizável, promovendo uma melhor organização do código.

### Impacto na Qualidade do Projeto:

- **Usabilidade Melhorada**: 
  - A nova interface proporciona uma experiência de usuário superior, com controles intuitivos e feedback visual claro sobre o status das operações.
  
- **Escalabilidade**: 
  - O projeto agora está mais preparado para futuras expansões e adições de funcionalidades, graças à estrutura modular e ao uso de tecnologias modernas.
  
- **Eficiência no Desenvolvimento**: 
  - As ferramentas e bibliotecas utilizadas, como React e Tailwind CSS, aumentaram a eficiência do desenvolvimento, permitindo uma implementação mais rápida e eficaz das funcionalidades necessárias.

Em resumo, a mudança para React foi uma decisão estratégica que elevou significativamente a qualidade e a usabilidade do nosso projeto. Continuaremos a buscar melhorias e inovações, garantindo que nossa interface permaneça alinhada com as melhores práticas e tendências atuais de desenvolvimento web.

