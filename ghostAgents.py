
# coding: utf-8

# In[1]:

# ghostAgents.py
# --------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

from game import Agent
from game import Actions
from game import Directions
import random
from util import manhattanDistance
import util

class GhostAgent( Agent ):
  def __init__( self, index ):
    self.index = index

  def getAction( self, state ):
    dist = self.getDistribution(state)
    if len(dist) == 0: 
      return Directions.STOP
    else:
      return util.chooseFromDistribution( dist )
    
  def getDistribution(self, state):
    "Returns a Counter encoding a distribution over actions from the provided state."
    util.raiseNotDefined()

class RandomGhost( GhostAgent ):
  "A ghost that chooses a legal action uniformly at random."
  def getDistribution( self, state ):
    dist = util.Counter()
    for a in state.getLegalActions( self.index ): dist[a] = 1.0
    dist.normalize()
    return dist

class DirectionalGhost( GhostAgent ):
  "A ghost that prefers to rush Pacman, or flee when scared."
  def __init__( self, index, prob_attack=0.8, prob_scaredFlee=0.8 ):
    self.index = index
    self.prob_attack = prob_attack
    self.prob_scaredFlee = prob_scaredFlee
      
  def getDistribution( self, state ):
    # Read variables from state
    ghostState = state.getGhostState( self.index )
    legalActions = state.getLegalActions( self.index )
    pos = state.getGhostPosition( self.index )
    isScared = ghostState.scaredTimer > 0
    
    speed = 1
    if isScared: speed = 0.5
    
    actionVectors = [Actions.directionToVector( a, speed ) for a in legalActions]
    newPositions = [( pos[0]+a[0], pos[1]+a[1] ) for a in actionVectors]
    pacmanPosition = state.getPacmanPosition()

    # Select best actions given the state
    distancesToPacman = [manhattanDistance( pos, pacmanPosition ) for pos in newPositions]
    if isScared:
      bestScore = max( distancesToPacman )
      bestProb = self.prob_scaredFlee
    else:
      bestScore = min( distancesToPacman )
      bestProb = self.prob_attack
    bestActions = [action for action, distance in zip( legalActions, distancesToPacman ) if distance == bestScore]
    
    # Construct distribution
    dist = util.Counter()
    for a in bestActions: dist[a] = bestProb / len(bestActions)
    for a in legalActions: dist[a] += ( 1-bestProb ) / len(legalActions)
    dist.normalize()
    return dist

class aStarGhost( GhostAgent ):
  def __init__( self, index):
    self.index = index
  #gera os possiveis sucessores do fantasma
  def getGhostSuccessors(self, movimentosValidos, state):
      return [(state.generateSuccessor(self.index, action), action, state.generateSuccessor(self.index, action).getGhostPosition(self.index)) for action in movimentosValidos]
      
  def getDistribution(self, state):
    from util import PriorityQueue, Counter, manhattanDistance

    estadoGasparzinho = state.getGhostState(self.index)# pega o estado atual do fantasma
    movimentosValidos = state.getLegalActions(self.index)#armazena os movimentos validos no estado atual
    isScared = estadoGasparzinho.scaredTimer > 0#checa se o fantasma esta assustado do pacman
    posPacman = state.getPacmanPosition()#armazena a posicao atual do pacman
    caminho = Counter()#counter com os caminhos

    if not isScared:
      caminhosPossiveis = PriorityQueue()#fila de prioridade que armazenara os caminhos possiveis
      visitados = []#lista com os caminhos que já foram visitados
      caminhosPossiveis.push((state,[]) , 0)

      for estados in state.getGhostPositions():# faz com que o caminho dos fantasmas já seja contabilizado, para que cerquem o pacman ao ives de ambos sempre seguirem o mesmo caminho
        if estados != state.getGhostPosition(self.index): visitados.append(estados)
      
      while not caminhosPossiveis.isEmpty():
        estadoAtual , sequenciaMovimentos = caminhosPossiveis.pop()#pega o primeiro caminho
        
        for proxEstado, proxMovimento, proxPosicao in self.getGhostSuccessors(estadoAtual.getLegalActions(self.index), estadoAtual):
            
          if not proxEstado.getGhostPosition(self.index) in visitados:#checa se o proximo estado ja foi visitado
            
            if proxEstado.getGhostPosition(self.index) == posPacman:#checa se o fantasmas alcancou o pacman
                
              if len(sequenciaMovimentos): #faz o fantasma se mover pelo caminho
                caminho[sequenciaMovimentos[0]] = 1
                
              else: 
                caminho[state.getLegalActions(self.index)[0]] = 1
                
              return caminho
        
            movimentoRealizado =  sequenciaMovimentos + [proxMovimento]#computa o movimento realizado e o adiciona na lista de estados visitados
            caminhosPossiveis.push((proxEstado, movimentoRealizado), len(movimentoRealizado) + manhattanDistance(proxPosicao, posPacman))
            visitados.append(proxPosicao)
     
    rotaFuga = self.getGhostSuccessors(movimentosValidos, state)#fuga do fantasma, caso ele esteja com medo do pacman
    _, correr = max((manhattanDistance(rota[2], posPacman), rota[1]) for rota in rotaFuga) 
    caminho[correr] = 1
    return caminho


# In[ ]:



