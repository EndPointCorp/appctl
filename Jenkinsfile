pipeline {
  agent {
    dockerfile {
      args "-u 0 -v /var/lib/jenkins/.ssh:/root/ssh"
    }
  }
  environment {
    APTLY_SERVER = credentials('aptly-server-url')
  }
  stages {
    stage('Test') {
      steps {
        sh "./scripts/run_tests.sh"
      }
    }
    stage('Build') {
      when {
        branch 'master'
      }
      steps {
        input('debug this step')
        sh "./pack-debs"
      }
    }
    stage('Deploy') {
      when {
        branch 'mater'
      }
      steps {
        sh "./scripts/deploy.sh"
      }
    }
  }
}
